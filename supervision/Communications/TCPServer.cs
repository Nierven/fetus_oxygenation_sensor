using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Diagnostics;
using System.Collections.Generic;

namespace Communications
{
    /// <summary>
    /// Serveur utilisable pour communiquer avec plusieurs systèmes sur différents IPs et ports.
    /// </summary>
    public class TCPServer
    {
        #region Constructors

        /// <summary>Crée un serveur (non démarré par défaut) vide.</summary>
        public TCPServer() { }

        /// <summary>Crée un serveur (non démarré par défaut) attendant une connexion sur un port.</summary>
        /// <param name="port">Port sur lequel attendre une connexion entrante.</param>
        public TCPServer(int port) => AddEntryPoint(port);

        #endregion
        #region Properties & Fields

        /// <summary>Liste des connexions ouvertes sur le serveur.</summary>
        public List<TCPClient> Clients { get; } = new List<TCPClient>();

        /// <summary>Définit si le serveur attend constamment des connexions entrantes sur les ports spécifiés.</summary>
        public bool ContineousListening { get; set; } = true;

        /// <summary>Résolu lorsqu'un nouveau client est ajouté à Clients.</summary>
        public event NewClientEventHandler NewClient;

        private readonly List<CustomTcpListener> _tcpListeners = new List<CustomTcpListener>();

        #endregion
        #region Control

        /// <summary>
        /// Paramètre une nouvelle entrée pour une connexion entrante.
        /// </summary>
        /// <param name="port">Port sur lequel attendre une connexion entrante.</param>
        public string AddEntryPoint(int port)
        {
            var addressIp = IP.GetLocalIP();
            _tcpListeners.Add(new CustomTcpListener(addressIp, port));
            Trace.WriteLine($"Listener address: {addressIp}:{port}");

            return $"{addressIp}:{port}";
        }

        private void StartListening(object obj)
        {
            var index = (int)obj;
            var tcpListener = _tcpListeners[index];

            if (!tcpListener.IsListening)
            {
                try
                {
                    tcpListener.Start();
                    Trace.WriteLine($"Listening on {tcpListener.LocalEndpoint}...");
                    tcpListener.IsListening = true;

                    // Attend une connexion entrante
                    TcpClient tcpClient = tcpListener.AcceptTcpClient();

                    Trace.WriteLine($"New client using: {tcpClient.Client.RemoteEndPoint}");

                    // Ajoute un Client utilisant le nouveau client Tcp
                    TCPClient newClient = new TCPClient(tcpClient);
                    Clients.Add(newClient);

                    NewClient?.Invoke(this, new NewClientEventArgs(newClient));
                    tcpListener.IsListening = false;

                    // Redémarre l'écoute si spécifié
                    if (ContineousListening)
                        StartListening(index);
                }
                catch (SocketException ex) when (ex.SocketErrorCode == SocketError.AddressAlreadyInUse)
                {
                    EndPoint endPoint = tcpListener.LocalEndpoint;
                    _tcpListeners.RemoveAt(index);

                    Trace.WriteLine($"Duplicated listener for {endPoint} destroyed.");
                }
                catch (SocketException ex) when (ex.SocketErrorCode == SocketError.Interrupted)
                {
                    EndPoint endPoint = tcpListener.LocalEndpoint;
                    Trace.WriteLine($"Listener for {endPoint} closed.");
                }
            }
        }

        #endregion

        #region Start & Close

        /// <summary>
        /// Ferme le serveur et les clients associées.
        /// </summary>
        ~TCPServer() => Close();

        /// <summary>
        /// Ferme le serveur et les clients associés.
        /// </summary>
        public void Close()
        {
            foreach (TCPClient client in Clients)
                client.Close();

            foreach (CustomTcpListener tcpListener in _tcpListeners)
                tcpListener.Stop();
        }

        /// <summary>
        /// Démarre le serveur et attend des connexions entrantes sur les ports précédemment spécifiées.
        /// </summary>
        public void Start()
        {
            for (int i = 0; i < _tcpListeners.Count; i++)
                new Thread(StartListening)
                {
                    Name = $"ServerListeningThread for {_tcpListeners[i].LocalEndpoint}",
                    Priority = ThreadPriority.Normal
                }.Start(i);
        }

        #endregion
    }

    #region CustomTcpListener

    internal class CustomTcpListener : TcpListener
    {
        public CustomTcpListener(IPEndPoint localEp) : base(localEp) { }
        public CustomTcpListener(IPAddress localaddr, int port) : base(localaddr, port) { }

        public bool IsListening { get; set; } = false;
    }

    #endregion
    #region NewClientEvent

    /// <summary>
    /// EventArgs fournissant le client concernant par l'événement.
    /// </summary>
    public class NewClientEventArgs : EventArgs
    {
        /// <summary>
        /// Crée une instance de <see cref="NewClientEventArgs"/>.
        /// </summary>
        /// <param name="client">Client concerné.</param>
        public NewClientEventArgs(TCPClient client) => NewClient = client;

        /// <summary>
        /// Nouveau client créé par un <see cref="TCPServer"/>.
        /// </summary>
        public TCPClient NewClient { get; }
    }

    /// <summary>
    /// Event de modification de client avec un argument contenant le client modifié.
    /// </summary>
    public delegate void NewClientEventHandler(object sender, NewClientEventArgs e);

    #endregion
}
