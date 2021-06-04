using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;
using Communications.Annotations;

namespace Communications
{
    /// <summary>
    /// Objet associable à un port série ou un client réseau permettant le traitement de données selon les protocoles établis par le RCT et la RoboCup.
    /// </summary>
    public sealed class MessageStream : INotifyPropertyChanged
    {
        #region Constructors

        /// <summary>
        /// Crée une instance de <see cref="MessageStream"/> vide.
        /// </summary>
        public MessageStream() { }
        /// <summary>
        /// Crée une instance de <see cref="MessageStream"/> avec un stream par défaut.
        /// </summary>
        /// <param name="usedStream">Stream à utiliser.</param>
        public MessageStream(IStream usedStream) => UsedStream = usedStream;

        #endregion
        #region Streams

        private IStream _innerStream;

        /// <summary>
        /// Flux de données utilisé.
        /// Attention : le changer revient à supprimer tous les messages en attente de traitement.
        /// </summary>
        public IStream UsedStream
        {
            get => _innerStream;
            set
            {
                _innerStream?.Close();
                _innerStream = value;

                if (value != null)
                {
                    new Thread(() =>
                    {
                        value.PropertyChanged += (sender, e) =>
                        {
                            if (e.PropertyName == "IsConnected")
                                OnPropertyChanged(nameof(IsOpen));
                        };

                        // Ferme l'ancien flux et démarre le nouveau
                        _innerStream.Start();

                        // Souscrit à DataReceivedEvent
                        _innerStream.DataReceivedEvent += IStream_DataReceivedEvent;
                        _waiting = true;

                        OnPropertyChanged();
                    }).Start();
                }
                else
                {
                    OnPropertyChanged();
                }
            }
        }

        #endregion
        #region Properties & Fields

        /// <summary>Indique si <see cref="UsedStream"/> est actuellement ouvert ou non.</summary>
        public bool IsOpen => UsedStream?.IsConnected ?? false;

        private bool _usesRobocupCharacterBasedProtocol = false;

        /// <summary>Type de protocole utilisé sur ce flux.</summary>
        public bool UsesRobocupCharacterBasedProtocol
        {
            get => _usesRobocupCharacterBasedProtocol;
            set
            {
                _usesRobocupCharacterBasedProtocol = value;
                OnPropertyChanged();
            }
        }

        /// <summary>Instant de réception du dernier message reçu.</summary>
        public DateTime LastReceivedMessageTime { get; private set; }

        private void OnDecodedMessage(Message message)
        {
            LastReceivedMessageTime = DateTime.Now;

            OnPropertyChanged(nameof(LastReceivedMessageTime));
            MessageDecoded?.Invoke(this, new MessageDecodedEventArgs(message));
        }

        /// <summary>Event de notification de propriété changée pour Binding.</summary>
        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        private void OnPropertyChanged([CallerMemberName] string propertyName = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        /// <summary>Résolu lorsqu'un nouveau message est reçu.</summary>
        public event MessageDecodedEventHandler MessageDecoded;

        /// <summary></summary>
        public int MaximumPayloadLength { get; set; } = 1024;

        #endregion

        #region Sender

        private bool SendMessage(Message message, Action<byte[]> writeDelegate)
        {
            // Si MessageStream utilise le protocole de la Robocup, on applique automatiquement aux messages à envoyer ce protocole.
            if (UsesRobocupCharacterBasedProtocol)
                message.ProtocolUsed = Protocol.CharacterBased;

            // On obtient le message sous forme d'array d'octets et on l'envoie
            if (IsOpen)
            {
                try
                {
                    writeDelegate(message.GetBytes());
                    return true;
                }
                catch
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// Envoie un message sur le flux utilisé.
        /// </summary>
        /// <param name="message">Message à envoyer.</param>
        public bool SendMessage(Message message)
            => IsOpen && SendMessage(message, UsedStream.WriteAsync);

        /// <summary>
        /// Envoie un message sur le flux utilisé et attend la fin de la transmission.
        /// </summary>
        /// <param name="message">Message à envoyer.</param>
        public bool SendMessageSync(Message message) 
            => IsOpen && SendMessage(message, UsedStream.Write);

        #endregion
        #region Receiver

        /// <summary>
        /// Réceptionne les octets d'un flux et les décode.
        /// </summary>
        private void IStream_DataReceivedEvent(object sender, EventArgs e)
        {
            IStream stream = sender as IStream;
            while (stream.BytesToRead > 0)
                DecodeMessage(stream.ReadByte());
        }

        #endregion
        #region Decoding

        private bool _waiting = true;
        private Protocol _actualInternalProtocol;

        // Données décodées
        private int _decodedCommand;
        private int _decodedPayloadLength;
        private int _decodedPayloadIndex;
        private byte[] _decodedPayload;

        /// <summary>
        /// Decode l'octet envoyé selon le protocole utilisé.
        /// </summary>
        public void DecodeMessage(byte b)
        {
            // Si on utilise le protocole de la Robocup, décoder directement le caractère
            if (UsesRobocupCharacterBasedProtocol)
            {
                DecodeCharacterBasedMessage(b);
            }
            else if (_waiting)
            {
                switch (b)
                {
                    case 0xFD:
                        _actualInternalProtocol = Protocol.Short;
                        _rcvStateShort = ShortStateReception.CommandMSB;
                        _waiting = false;
                        break;
                    case 0xFE:
                        _actualInternalProtocol = Protocol.Normal;
                        _rcvStateNormal = NormalStateReception.CommandMSB;
                        _waiting = false;
                        break;
                }
            }
            else
            {
                switch (_actualInternalProtocol)
                {
                    case Protocol.Short:
                        DecodeShortMessage(b);
                        break;
                    case Protocol.Normal:
                        DecodeNormalMessage(b);
                        break;
                }
            }
        }

        #region CharacterBased

        private void DecodeCharacterBasedMessage(byte b)
        {
            OnDecodedMessage(new Message
            {
                Command = (Commands) b,
                Payload = null,
                ProtocolUsed = Protocol.CharacterBased
            });
        }

        #endregion

        #region Short

        private enum ShortStateReception
        {
            CommandMSB,
            CommandLSB,
            Checksum
        }

        private ShortStateReception _rcvStateShort = ShortStateReception.CommandMSB;

        private void DecodeShortMessage(byte b)
        {
            switch (_rcvStateShort)
            {
                case ShortStateReception.CommandMSB:
                    _decodedCommand = b << 8;
                    _rcvStateShort = ShortStateReception.CommandLSB;
                    break;
                case ShortStateReception.CommandLSB:
                    _decodedCommand += b;
                    _rcvStateShort = ShortStateReception.Checksum;
                    break;
                case ShortStateReception.Checksum:
                    Message message = new Message((Commands) _decodedCommand);

                    if (message.CalculateChecksum() == b)
                        OnDecodedMessage(message);
                    else Trace.WriteLine("Checksum error.");

                    _rcvStateShort = ShortStateReception.CommandMSB;
                    _waiting = true;

                    break;
                default:
                    _rcvStateShort = ShortStateReception.CommandMSB;
                    _waiting = true;

                    break;
            }
        }

        #endregion

        #region Normal

        private enum NormalStateReception
        {
            CommandMSB,
            CommandLSB,
            PayloadLengthMSB,
            PayloadLengthLSB,
            Payload,
            Checksum
        }

        private NormalStateReception _rcvStateNormal = NormalStateReception.CommandMSB;

        private void DecodeNormalMessage(byte b)
        {
            switch (_rcvStateNormal)
            {
                case NormalStateReception.CommandMSB:
                    _decodedCommand = b << 8;
                    _rcvStateNormal = NormalStateReception.CommandLSB;
                    break;
                case NormalStateReception.CommandLSB:
                    _decodedCommand += b;
                    _rcvStateNormal = NormalStateReception.PayloadLengthMSB;
                    break;
                case NormalStateReception.PayloadLengthMSB:
                    _decodedPayloadLength = b << 8;
                    _rcvStateNormal = NormalStateReception.PayloadLengthLSB;
                    break;
                case NormalStateReception.PayloadLengthLSB:
                    _decodedPayloadLength += b;

                    if (_decodedPayloadLength == 0)
                    {
                        _rcvStateNormal = NormalStateReception.CommandMSB;
                        _waiting = true;
                        break;
                    }
                    else if (_decodedPayloadLength >= MaximumPayloadLength)
                    {
                        _rcvStateNormal = NormalStateReception.CommandMSB;
                        _waiting = true;
                        break;
                    }

                    _decodedPayload = new byte[_decodedPayloadLength];
                    _decodedPayloadIndex = 0;
                    _rcvStateNormal = NormalStateReception.Payload;

                    break;
                case NormalStateReception.Payload:
                    _decodedPayload[_decodedPayloadIndex++] = b;
                    if (_decodedPayloadIndex >= _decodedPayloadLength)
                        _rcvStateNormal = NormalStateReception.Checksum;

                    break;
                case NormalStateReception.Checksum:
                    Message message = new Message
                    {
                        Command = (Commands) _decodedCommand,
                        Payload = _decodedPayload
                    };

                    if (message.CalculateChecksum() == b)
                        OnDecodedMessage(message);
                    else Trace.WriteLine("Checksum error.");

                    _rcvStateNormal = NormalStateReception.CommandMSB;
                    _waiting = true;

                    break;
                default:
                    _rcvStateNormal = NormalStateReception.CommandMSB;
                    _waiting = true;

                    break;
            }
        }

        #endregion

        #endregion

        #region Closing

        /// <summary>Ferme le flux.</summary>
        ~MessageStream() => Close();

        /// <summary>Ferme le flux.</summary>
        public void Close() => _innerStream?.Close();

        #endregion
    }

    /// <summary>Delegate de message décodé.</summary>
    public delegate void MessageDecodedEventHandler(object sender, MessageDecodedEventArgs e);

    /// <summary>Arguments d'event de message décodé.</summary>
    public class MessageDecodedEventArgs : EventArgs
    {
        /// <summary>Nouveau message décodé.</summary>
        public readonly Message MessageDecoded;

        /// <summary>Crée une instance de <see cref="MessageDecodedEventArgs"/>.</summary>
        public MessageDecodedEventArgs(Message messageDecoded)
            => MessageDecoded = messageDecoded;
    }
}
