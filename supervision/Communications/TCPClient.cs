using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Runtime.CompilerServices;
using System.Threading;
using Communications.Annotations;

namespace Communications
{
    /// <summary>
    /// Client TCP permettant un échange facile et asynchrone d'octets.
    /// </summary>
    public sealed class TCPClient : IStream, IDisposable, INotifyPropertyChanged
    {
        #region Constructors

        /// <summary>
        /// Instancie un client avec une adresse IP de destination et un port à utiliser.
        /// </summary>
        /// <param name="addressIp">Addresse IP à adresser.</param>
        /// <param name="port">Port à utiliser.</param>
        public TCPClient(string addressIp, int port)
        {
            AddressIp = addressIp;
            Port = port;

            _readThread = new Thread(Connect)
            {
                Name = $"ClientThread for {AddressIp}:{Port}"
            };
        }

        internal TCPClient(TcpClient tcpClient)
        {
            // Ajoute le client spécifié, en obtient son flux puis crée le buffer de réception
            _tcpClient = tcpClient;

            AddressIp = ((IPEndPoint) _tcpClient.Client.RemoteEndPoint).Address.ToString();
            Port = ((IPEndPoint) tcpClient.Client.RemoteEndPoint).Port;

            _readThread = new Thread(Connect)
            {
                Name = $"ClientThread for {AddressIp}:{Port}"
            };
        }

        #endregion
        #region Properties & Fields

        private readonly Queue<byte> _bytes = new Queue<byte>();
        private readonly Thread _readThread;

        private TcpClient _tcpClient;
        private bool _closing = false;

        /// <summary>Indique si un appel de Start lance une recherche infinie.</summary>
        public bool ContineousRead { get; set; } = true;

        /// <summary>Indique si le <see cref="TCPClient"/> doit continuer de tenter de se connecter.</summary>
        public bool KeepsRetryConnection { get; set; } = true;

        /// <summary>Adresse IP à adresser.</summary>
        public string AddressIp { get; private set; }

        /// <summary>Port utilisé pour les communications.</summary>
        public int Port { get; private set; }

        /// <summary>Indique le nombre de données disponibles à la lecture.</summary>
        public int BytesToRead => _bytes.Count;

        /// <summary>Indique si le client est correctement connecté.</summary>
        public bool IsConnected => _tcpClient?.Connected ?? false;

        #endregion
        #region Events

        /// <summary>Résolu lorsque des données sont reçues par le client.</summary>
        public event EventHandler DataReceivedEvent;

        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        private void OnPropertyChanged([CallerMemberName] string propertyName = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        #endregion

        #region Control

        /// <summary>Permet de démarrer ce <see cref="TCPClient"/>.</summary>
        public void Start()
        {
            if (!_readThread.IsAlive)
                _readThread.Start();
        } 

        private void Connect()
        {
            if (Port != 0 && !string.IsNullOrWhiteSpace(AddressIp))
            {
                if (!IsConnected)
                {
                    // Ferme le client et le flux réseau si nécessaire
                    _tcpClient?.Close();
                    OnPropertyChanged(nameof(IsConnected));

                    do
                    {
                        if (_closing)
                        {
                            // Arrête l'exécution du thread si demandé
                            _closing = false;
                            return;
                        }

                        try
                        {
                            // Se connecte au RemoteEndPoint spécifié
                            _tcpClient = new TcpClient(AddressIp, Port);
                            OnPropertyChanged(nameof(IsConnected));

                            if (IsConnected)
                                break;
                        }
                        catch { if (KeepsRetryConnection && !_closing) Thread.Sleep(500); else return; }
                    } while (KeepsRetryConnection);

                    if (!IsConnected)
                        // Arrête si toujours pas connecté
                        return;
                }

                ReadClient();
            }
        }

        #endregion
        #region Input

        private void ReadClient()
        {
            while (ContineousRead && !_closing)
            {
                byte[] buffer = new byte[_tcpClient.ReceiveBufferSize];
                int numberOfBytesReceived = 0;

                try
                {
                    // Obtient les nouvelles données reçues ainsi que leur nombre
                    numberOfBytesReceived = _tcpClient.GetStream().Read(buffer, 0, buffer.Length);
                }
                catch (ObjectDisposedException) { }
                catch (InvalidOperationException) { }
                catch (IOException ex)
                {
                    Trace.WriteLine(ex.Message);
                }

                if (numberOfBytesReceived == 0)
                {
                    // Si pas de données, s'arrêter
                    Close();
                    return;
                }

                // Ajoute les données reçues à la Queue
                for (int i = 0; i < numberOfBytesReceived; i++)
                    _bytes.Enqueue((byte)buffer[i]);

                DataReceivedEvent?.Invoke(this, EventArgs.Empty);
            }
        }

        /// <summary>Lit un octet dans le buffer de réception du client.</summary>
        public byte ReadByte() => _bytes.Dequeue();

        #endregion
        #region Output

        /// <summary>
        /// Envoie du texte sur le port ouvert.
        /// </summary>
        /// <param name="bytes">Message à envoyer</param>
        public void Write(byte[] bytes) => _tcpClient.GetStream().Write(bytes, 0, bytes.Length);
        
        /// <summary>
        /// Envoie du texte sur le port ouvert de manière asynchrone.
        /// </summary>
        /// <param name="bytes">Message à envoyer</param>
        public void WriteAsync(byte[] bytes) => _tcpClient.GetStream().WriteAsync(bytes, 0, bytes.Length);

        #endregion

        #region Closing

        /// <summary>Déconnecte le client.</summary>
        public void Close() => Dispose();

        private bool _disposedValue = false; // To detect redundant calls
        /// <summary>Détruit le client.</summary>
        private void Dispose(bool disposing)
        {
            if (!_disposedValue)
            {
                if (disposing)
                {
                    // Aucune ressource managée spécifique à supprimer.
                }

                _closing = true;
                _tcpClient?.Dispose();

                OnPropertyChanged(nameof(IsConnected));

                _disposedValue = true;
            }
        }

        /// <summary>Déconnecte le client.</summary>
        ~TCPClient() => Dispose(false);

        /// <summary>Détruit le client.</summary>
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        #endregion
    }
}
