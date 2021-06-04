using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Diagnostics;
using System.Collections.Generic;
using System.Text;

namespace Communications
{
    /// <summary>
    /// Serveur utilisable pour communiquer avec plusieurs systèmes sur différents IPs et ports.
    /// </summary>
    public class UDPServer
    {
        #region Constructors

        /// <summary>Crée un serveur (non démarré par défaut) vide.</summary>
        public UDPServer() { }

        /// <summary>Crée un serveur (non démarré par défaut) attendant une connexion sur un port.</summary>
        /// <param name="port">Port sur lequel attendre une connexion entrante.</param>
        public UDPServer(int port) => AddEntryPoint(port);

        #endregion
        #region Properties & Fields

        /// <summary>Liste des connexions ouvertes sur le serveur.</summary>
        public List<UDPClient> Clients { get; } = new List<UDPClient>();

        /// <summary>Définit si le serveur attend constamment des connexions entrantes sur les ports spécifiés.</summary>
        public bool ContineousListening { get; set; } = true;

        /// <summary>Résolu lorsqu'un nouveau client est ajouté à Clients.</summary>
        public event NewUDPClientEventHandler NewClient;

        private readonly List<UdpListener> _udpListeners = new List<UdpListener>();

        #endregion

        #region Control

        /// <summary>
        /// Paramètre une nouvelle entrée pour une connexion entrante.
        /// </summary>
        /// <param name="port">Port sur lequel attendre une connexion entrante.</param>
        public string AddEntryPoint(int port)
        {
            _udpListeners.Add(new UdpListener(port));
            IPEndPoint localIpEndPoint = _udpListeners[_udpListeners.Count - 1].LocalEndpoint;
            Trace.WriteLine($"Listener address: {localIpEndPoint}");

            return localIpEndPoint.ToString();
        }

        private void StartListening(object obj)
        {
            var index = (int)obj;
            var udpListener = _udpListeners[index];

            if (!udpListener.IsListening)
            {
                try
                {
                    // Attend une connexion entrante
                    Trace.WriteLine($"Listening on {udpListener.LocalEndpoint}...");
                    UdpClient udpClient = udpListener.AcceptUdpClient();

                    Trace.WriteLine($"New client using: {udpClient.Client.RemoteEndPoint}");

                    // Ajoute un Client utilisant le nouveau client UDP
                    UDPClient newClient = new UDPClient(udpClient);
                    Clients.Add(newClient);

                    NewClient?.Invoke(this, new NewUDPClientEventArgs(newClient));

                    // Redémarre l'écoute si spécifié
                    if (ContineousListening)
                        StartListening(index);
                }
                catch (SocketException ex) when (ex.SocketErrorCode == SocketError.AddressAlreadyInUse)
                {
                    EndPoint endPoint = udpListener.LocalEndpoint;
                    _udpListeners.RemoveAt(index);

                    Trace.WriteLine($"Duplicated listener for {endPoint} destroyed.");
                }
                catch (Exception ex)
                {
                    Trace.WriteLine(ex.Message);
                }
            }
        }

        #endregion
        #region Start & Close

        /// <summary>
        /// Ferme le serveur et les clients associées.
        /// </summary>
        ~UDPServer() => Close();

        /// <summary>
        /// Ferme le serveur et les clients associés.
        /// </summary>
        public void Close()
        {
            foreach (UDPClient client in Clients)
                client?.Close();

            foreach (UdpListener udpListener in _udpListeners)
                udpListener.Stop();
        }

        /// <summary>
        /// Démarre le serveur et attend des connexions entrantes sur les ports précédemment spécifiées.
        /// </summary>
        public void Start()
        {
            for (int i = 0; i < _udpListeners.Count; i++)
                new Thread(StartListening)
                {
                    Name = $"ServerListeningThread for {_udpListeners[i].LocalEndpoint}",
                    Priority = ThreadPriority.Normal
                }.Start(i);
        }

        #endregion
    }

    #region CustomUdpListener

    internal class UdpListener
    {
        private readonly UdpClient _client;
        private readonly int _port;

        public UdpListener(int port)
        {
            _port = port;
            _client = new UdpClient(_port);
            LocalEndpoint = new IPEndPoint(IP.GetLocalIP(), _port);
        }

        public UdpClient AcceptUdpClient()
        {
            byte[] answer;
            UdpClient client;

            IsListening = true;

            do
            {
                IPEndPoint iPEndPoint;
                byte[] buffer;

                do
                {
                    // Attends une demande de connexion (envoi de UDPConnectionAsked)
                    iPEndPoint = new IPEndPoint(IPAddress.Any, _port);
                    buffer = _client.Receive(ref iPEndPoint);
                }
                while (Encoding.ASCII.GetString(buffer) != "UDPConnectionAsked");

                // Connecte un nouveau client UDP au point qui demande une connexion
                client = new UdpClient();
                client.Connect(iPEndPoint);
                int localPort = (client.Client.LocalEndPoint as IPEndPoint).Port;

                // Obtient le port local du client et l'envoie au client destinataire
                _client.Send(new[] { (byte)(localPort >> 8), (byte)localPort }, 2, iPEndPoint);

                // Attends une réponse pour finaliser la connexion
                answer = _client.Receive(ref iPEndPoint);
            }
            while (Encoding.ASCII.GetString(answer) != "ACK");

            IsListening = false;
            return client;
        }

        public void Stop() => _client.Close();

        public bool IsListening { get; private set; } = false;
        public IPEndPoint LocalEndpoint { get; private set; }
    }

    #endregion
    #region NewUDPClientEvent

    /// <summary>
    /// EventArgs fournissant le client concernant par l'événement.
    /// </summary>
    public class NewUDPClientEventArgs : EventArgs
    {
        /// <summary>
        /// Crée une instance de <see cref="NewUDPClientEventArgs"/>.
        /// </summary>
        /// <param name="client">Client concerné.</param>
        public NewUDPClientEventArgs(UDPClient client) => NewClient = client;

        /// <summary>
        /// Client concerné par l'événement.
        /// </summary>
        public UDPClient NewClient { get; }
    }

    /// <summary>
    /// Event de modification de client avec un argument contenant le client modifié.
    /// </summary>
    public delegate void NewUDPClientEventHandler(object sender, NewUDPClientEventArgs e);

    #endregion
}
