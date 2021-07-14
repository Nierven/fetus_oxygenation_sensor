using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Management;
using System.Text.RegularExpressions;
using System.Threading;

namespace Communications
{
    /// <summary>
    /// Port série avancé utilisant l'interface IStream pour une utilisation simplifiée.
    /// </summary>
    public class AdvancedSerialPort : SerialPort, IStream, IDisposable
    {
        private Thread _startThread;

        /// <summary>Crée une instance de <see cref="AdvancedSerialPort"/> avec le baudrate et le port spécifiés.</summary>
        /// <param name="portName"></param>
        /// <param name="baudrate"></param>
        public AdvancedSerialPort(string portName, int baudrate) : base(portName, baudrate)
            => _isInitialized = true;

        /// <summary>
        /// Crée une instance de <see cref="AdvancedSerialPort"/> et tente la connection à un appareil avec le nom de fabriquant indiqué.
        /// Si tentative échouée, tente avec un appareil alternatif. Si encore échoué, les retente à l'infini.
        /// </summary>
        public AdvancedSerialPort(string vendorName, string secondVendorName, int baudrate) : base("COM1", baudrate)
        {
            _startThread = new Thread(() =>
            {
                string portName = "";
                bool searchsForAlternative = false;

                while (true)
                {
                    portName = SearchPortName(searchsForAlternative ? secondVendorName : vendorName);
                    searchsForAlternative ^= true;

                    if (!string.IsNullOrWhiteSpace(portName))
                        break;
                    else if (_closeAsked)
                        return;
                    else
                        Thread.Sleep(200);
                }

                PortName = portName;
                _isInitialized = true;

                if (_startAsked)
                    Start();
            });
                
            _startThread.Start();
        }

        private string SearchPortName(string vendorName)
        {
            try
            {
                ManagementObjectSearcher searcher = new ManagementObjectSearcher("root\\CIMV2",
                                                                                 "SELECT * FROM Win32_PnPEntity");
                foreach (ManagementObject queryObj in searcher.Get())
                {
                    // Recherche du fabriquant dans le DeviceID
                    if (queryObj["DeviceID"] != null && queryObj["DeviceID"].ToString().Contains(vendorName))
                        if (queryObj["Caption"] != null)
                        {
                            // Recherche du port COM dans le nom
                            string textToSearch = queryObj["Caption"].ToString();
                            string pattern = @"\((COM[0-9]+?)\)"; // Regex pour la recherche des FTDI 232
                            Regex r = new Regex(pattern, RegexOptions.IgnoreCase | RegexOptions.Singleline);

                            Match m = r.Match(textToSearch);
                            if (m.Success)
                                return m.Groups[1].ToString();
                        }
                }

                return "";
            }
            catch { return ""; }
        }

        /// <summary>Démarre le port série.</summary>
        public void Start()
        {
            if (_isInitialized)
            {
                _closeAsked = false;

                _startThread = new Thread(() =>
                {
                    int cpt = 0;
                    while (!IsOpen && cpt < 10 - 1)
                    {
                        if (_closeAsked)
                            return;

                        try
                        {
                            Open();
                        }
                        catch (Exception ex)
                        {
                            cpt++;
                            Trace.WriteLine($"Essai {cpt + 1} : {ex.Message}");
                            Thread.Sleep(10);
                        }
                    }

                    if (!IsOpen)
                        return;

                    new Thread(Read).Start();
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsConnected)));
                });

                _startThread.Start();
            }
            else _startAsked = true;
        }

        /// <summary>Ferme le port série.</summary>
        void IStream.Close() => Close();
        private new void Close()
        {
            base.Close();

            _isInitialized = false;
            _startAsked = false;
            _closeAsked = true;

            _startThread?.Abort();
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("IsConnected"));
        }

        /// <summary>Indique si le port série est ouvert ou fermé.</summary>
        public bool IsConnected => IsOpen;

        /// <summary>Indique si port automatique trouvé.</summary>
        private bool _isInitialized = false;
        /// <summary>Indique si démarrage demandé (parce que port non encore trouvé).</summary>
        private bool _startAsked = false;
        /// <summary>Indique si arrêt demandé (parce que port non encore trouvé).</summary>
        private bool _closeAsked = false;

        /// <summary>Event de notification de propriété changée pour Binding.</summary>
        public event PropertyChangedEventHandler PropertyChanged;
        /// <summary>Résolu lorsque des données sont disponibles.</summary>
        public event EventHandler DataReceivedEvent;

        #region Reading/Writing

        private readonly Queue<byte> _bytes = new Queue<byte>();

        /// <summary>Callback de read sur le BaseStream du port série.</summary>
        private async void Read()
        {
            try
            {
                while (IsOpen)
                {
                    byte[] buffer = new byte[ReadBufferSize];
                    int availableBytes = await BaseStream.ReadAsync(buffer, 0, buffer.Length);

                    for (int i = 0; i < availableBytes; i++)
                        _bytes.Enqueue(buffer[i]);

                    DataReceivedEvent?.Invoke(this, EventArgs.Empty);
                }
            }
            catch (IOException ex)
            {
                Trace.WriteLine(ex.Message);
                Close();
            }
        }

        /// <summary>Btient le nombre d'octets disponibles en réception de la liaison série..</summary>
        int IStream.BytesToRead => _bytes.Count;
        /// <summary>Lit un octet du buffer de réception du port série.</summary>
        byte IStream.ReadByte() => _bytes.Dequeue();

        /// <summary>Écrit sur le port série un array d'octet.</summary>
        /// <param name="bytes">Array d'octet à envoyer.</param>
        public void Write(byte[] bytes) => BaseStream.Write(bytes, 0, bytes.Length);
        /// <summary>Écrit sur le port série un array d'octet de manière asynchrone.</summary>
        /// <param name="bytes">Array d'octet à envoyer.</param>
        public void WriteAsync(byte[] bytes) => BaseStream.WriteAsync(bytes, 0, bytes.Length);

        #endregion

        private bool _disposedValue = false; // To detect redundant calls
        
        private new void Dispose(bool disposing)
        {
            if (!_disposedValue)
            {
                if (disposing)
                {
                    // Aucune ressource managée spécifique à supprimer.
                }

                Close();
                _disposedValue = true;
            }
        }

        /// <summary>Déconnecte le client.</summary>
        ~AdvancedSerialPort() => Dispose(false);

        /// <summary>Détruit le client.</summary>
        public new void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }
    }
}
