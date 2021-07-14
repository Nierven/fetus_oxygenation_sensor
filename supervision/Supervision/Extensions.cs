using System;

namespace Supervision
{
    public static class Extensions
    {
        /// <summary>
        /// Gets a copy of an array range.
        /// </summary>
        /// <param name="b">Array to copy from.</param>
        /// <param name="index">Index from which the copy starts.</param>
        /// <param name="length">Number of values to copy.</param>
        public static T[] GetRange<T>(this T[] b, int index, int length)
        {
            T[] b_final = new T[length];
            Array.Copy(b, index, b_final, 0, length);

            return b_final;
        }

        /// <summary>
        /// Gets byte array front a <see cref="float"/>.
        /// </summary>
        /// <param name="f"><see cref="float"/> to obtain an array from.</param>
        public static unsafe byte[] GetBytes(this float f)
        {
            byte* f_ptr = (byte*)&f;

            byte[] bytes = new byte[4];
            for (int i = 0; i < 4; i++)
                bytes[i] = f_ptr[i];

            return bytes;
        }

        /// <summary>
        /// Gets byte array front a <see cref="double"/>.
        /// </summary>
        /// <param name="d"><see cref="double"/> to obtain an array from.</param>
        public static unsafe byte[] GetBytes(this double d)
        {
            byte* d_ptr = (byte*)&d;

            byte[] bytes = new byte[8];
            for (int i = 0; i < 8; i++)
                bytes[i] = d_ptr[i];

            return bytes;
        }

        /// <summary>
        /// Gets the corresponding <see cref="float"/> from a byte array.
        /// </summary>
        /// <param name="b">Byte array to obtain a <see cref="float"/> from.</param>
        public static unsafe float GetFloat(this byte[] b)
        {
            float f;
            fixed (byte* b_ptr = b)
                f = *(float*)b_ptr;

            return f;
        }

        /// <summary>
        /// Gets the corresponding <see cref="double"/> from a byte array.
        /// </summary>
        /// <param name="b">Byte array to obtain a <see cref="double"/> from.</param>
        public static unsafe double GetDouble(this byte[] b)
        {
            double d;
            fixed (byte* b_ptr = b)
                d = *(double*)b_ptr;

            return d;
        }
    }
}
