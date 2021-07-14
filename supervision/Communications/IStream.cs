using System;
using System.ComponentModel;

namespace Communications
{
    /// <summary>
    /// Interface utilisable pour des communications simples entre deux systèmes.
    /// </summary>
    public interface IStream : INotifyPropertyChanged
    {
        /// <summary>Démarre la liaison.</summary>
        void Start();
        /// <summary>Arrête la liaison.</summary>
        void Close();

        /// <summary>Indique si le flux est correctement ouvert ou connecté.</summary>
        bool IsConnected { get; }

        /// <summary>Envoie des octets sur la liaison.</summary>
        /// <param name="bytes">Array d'octets à envoyer.</param>
        void Write(byte[] bytes);

        /// <summary>Envoie des octets sur la liaison de manière asynchrone.</summary>
        /// <param name="bytes">Array d'octets à envoyer.</param>
        void WriteAsync(byte[] bytes);

        /// <summary>Résolu lorsque des données sont reçues par le client.</summary>
        event EventHandler DataReceivedEvent;

        /// <summary>Indique le nombre de données disponibles à la lecture.</summary>
        int BytesToRead { get; }
        /// <summary>Lit un octet réceptionné.</summary>
        byte ReadByte();
    }
}
