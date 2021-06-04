using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Net;
using System.Net.Sockets;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using Communications.Annotations;

namespace Communications
{
    /// <summary>
    /// Client TCP permettant un échange facile et asynchrone d'octets.
    /// </summary>
    public sealed class UDPClient : IStream, IDisposable, INotifyPropertyChanged
    {
        #region Constructors

        /// <summary>
        /// Instancie un client avec une adresse IP de destination et un port à utiliser.
        /// </summary>
        /// <param name="addressIp">Addresse IP à adresser.</param>
        /// <param name="port">Port à utiliser.</param>
        public UDPClient(string addressIp, int port)
        {
            RemoteAddressIp = addressIp;
            RemotePort = port;

            _remoteIpEndPoint = new IPEndPoint(IPAddress.Parse(RemoteAddressIp), RemotePort);
            _readingThread = new Thread(Connect) { Name = $"ClientThread for {_remoteIpEndPoint}" };
        }

        /// <summary>
        /// Instancie un client avec une adresse IP de destination et un port à utiliser.
        /// </summary>
        /// <param name="addressIp">Addresse IP à adresser.</param>
        /// <param name="port">Port à utiliser.</param>
        public UDPClient(IPAddress addressIp, int port)
        {
            RemoteAddressIp = addressIp.ToString();
            RemotePort = port;

            _remoteIpEndPoint = new IPEndPoint(addressIp, RemotePort);
            _readingThread = new Thread(Connect) { Name = $"ClientThread for {_remoteIpEndPoint}" };
        }

        /// <summary>
        /// Instancie un client avec un client UDP .NET préconfiguré.
        /// </summary>
        public UDPClient(UdpClient client)
        {
            _udpClient = client;
            _remoteIpEndPoint = _udpClient.Client.RemoteEndPoint as IPEndPoint;
            _localIpEndPoint = _udpClient.Client.LocalEndPoint as IPEndPoint;

            RemoteAddressIp = _remoteIpEndPoint.Address.ToString();
            RemotePort = _remoteIpEndPoint.Port;
            LocalPort = _localIpEndPoint.Port;

            IsConnected = true;

            _readingThread = new Thread(Connect) { Name = $"ClientThread for {_remoteIpEndPoint}" };
        }

        #endregion
        #region Properties & Fields

        private UdpClient _udpClient;
        private readonly Queue<byte> _bytes = new Queue<byte>();
        
        private readonly Thread _readingThread;
        private bool _closing = false;

        /// <summary>Indique si un appel de Start lance une recherche infinie.</summary>
        public bool ContineousRead { get; set; } = true;

        /// <summary>Indique si le client retente de se connecter après 500 ms.</summary>
        public bool KeepsRetryConnection { get; set; } = true;

        private IPEndPoint _remoteIpEndPoint;
        private IPEndPoint _localIpEndPoint;

        /// <summary>Définit l'adresse IP à adresser.</summary>
        public string RemoteAddressIp { get; private set; }
        /// <summary>Définit le port de destination utilisé pour les communications.</summary>
        public int RemotePort { get; private set; }
        /// <summary>Définit le port local utilisé pour les communications.</summary>
        public int LocalPort { get; private set; }

        /// <summary>Indique le nombre de données disponibles à la lecture.</summary>
        public int BytesToRead => _bytes.Count;

        private bool _isConnected = false;
        /// <summary>Indique l'état de connexion du client.</summary>
        public bool IsConnected
        {
            get => _isConnected;
            private set
            {
                _isConnected = value;
                OnPropertyChanged();
            }
        }

        /// <summary>Résolu lorsque des données sont reçues par le client.</summary>
        public event EventHandler DataReceivedEvent;

        /// <summary>Event de notification de propriété changée pour Binding.</summary>
        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        private void OnPropertyChanged([CallerMemberName] string propertyName = null) 
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        #endregion

        #region Control

        /// <summary>Permet de lancer un thread qui cherchera de nouveaux octets envoyés au client.</summary>
        public void Start() => _readingThread.Start();

        private void Connect()
        {
            if (RemotePort != 0 && !string.IsNullOrWhiteSpace(RemoteAddressIp))
            {
                while (!IsConnected && !_closing)
                {
                    _udpClient?.Close();
                    try
                    {
                        _udpClient = new UdpClient();
                        _udpClient.Connect(_remoteIpEndPoint);
                        _localIpEndPoint = _udpClient.Client.LocalEndPoint as IPEndPoint;

                        byte[] connectionMsg = Encoding.ASCII.GetBytes("UDPConnectionAsked");
                        _udpClient.Send(connectionMsg, connectionMsg.Length);
                        byte[] receivedConnectionPort = _udpClient.Receive(ref _remoteIpEndPoint);

                        byte[] ackMsg = Encoding.ASCII.GetBytes("ACK");
                        _udpClient.Send(ackMsg, ackMsg.Length);

                        _remoteIpEndPoint.Port = (receivedConnectionPort[0] << 8) + receivedConnectionPort[1];
                        _udpClient.Close();

                        _udpClient = new UdpClient(_localIpEndPoint);
                        _udpClient.Connect(_remoteIpEndPoint);

                        IsConnected = true;
                    }
                    catch { if (KeepsRetryConnection && !_closing) Thread.Sleep(500); else return; }
                }

                if (_closing)
                    return;

                byte[] buffer = new byte[_udpClient.Client.ReceiveBufferSize];
                _udpClient.BeginReceive(ReadClient, buffer);
            }
        }

        #endregion
        #region Reading

        private void ReadClient(IAsyncResult iar)
        {
            if (!_closing)
            {
                byte[] buffer = _udpClient.EndReceive(iar, ref _remoteIpEndPoint);
                if (buffer.Length > 0)
                {
                    foreach (byte b in buffer)
                        _bytes.Enqueue(b);
                    DataReceivedEvent?.Invoke(this, EventArgs.Empty);
                }

                // Redémarre la lecture si lecture continue spécifiée
                if (ContineousRead && !_closing)
                    _udpClient.BeginReceive(new AsyncCallback(ReadClient), buffer);
            }
        }

        /// <summary>Lit un octet dans le buffer de réception du client.</summary>
        public byte ReadByte() => _bytes.Dequeue();

        #endregion
        #region InputOutput

        /// <summary>
        /// Envoie du texte sur le port ouvert.
        /// </summary>
        /// <param name="bytes">Message à envoyer</param>
        public void Write(byte[] bytes) => _udpClient.Send(bytes, bytes.Length);
        
        /// <summary>
        /// Envoie du texte sur le port ouvert de manière asynchrone.
        /// </summary>
        /// <param name="bytes">Message à envoyer</param>
        public void WriteAsync(byte[] bytes) => _udpClient.SendAsync(bytes, bytes.Length);

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
                _udpClient?.Dispose();
                IsConnected = false;

                _disposedValue = true;
            }
        }

        /// <summary>Déconnecte le client.</summary>
        ~UDPClient() => Dispose(false);

        /// <summary>Détruit le client.</summary>
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        #endregion
    }
}
