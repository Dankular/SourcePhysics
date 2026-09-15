namespace SourcePhysics;

/// <summary>Port of Valve's vstdlib CUniformRandomStream.</summary>
public sealed class SourceUniformRandomStream
{
    private const int Ia = 16807;
    private const int Im = 2147483647;
    private const int Iq = 127773;
    private const int Ir = 2836;
    private const int Ntab = 32;
    private const int Ndiv = 1 + (Im - 1) / Ntab;
    private const float Am = 1f / Im;
    private const float Rnmx = 1f - 1.2e-7f;
    private const uint MaxRandomRange = 0x7FFFFFFFu;

    private readonly int[] iv = new int[Ntab];
    private int idum;
    private int iy;

    public SourceUniformRandomStream(int seed = 0) => SetSeed(seed);

    public void SetSeed(int seed)
    {
        idum = seed < 0 ? seed : -seed;
        iy = 0;
    }

    public float RandomFloat(float low = 0f, float high = 1f)
    {
        var value = Am * GenerateRandomNumber();
        if (value > Rnmx) value = Rnmx;
        return value * (high - low) + low;
    }

    public float RandomFloatExp(float low, float high, float exponent)
    {
        var value = Am * GenerateRandomNumber();
        if (value > Rnmx) value = Rnmx;
        if (exponent != 1f) value = MathF.Pow(value, exponent);
        return value * (high - low) + low;
    }

    public int RandomInt(int low, int high)
    {
        if (low > high) throw new ArgumentOutOfRangeException(nameof(low));
        var range = (ulong)((long)high - low + 1L);
        if (range <= 1UL) return low;
        if (range - 1UL > MaxRandomRange) throw new ArgumentOutOfRangeException(nameof(high));
        var maxAcceptable = MaxRandomRange - ((MaxRandomRange + 1UL) % range);
        uint value;
        do value = unchecked((uint)GenerateRandomNumber());
        while (value > maxAcceptable);
        return low + (int)(value % range);
    }

    private int GenerateRandomNumber()
    {
        if (idum <= 0 || iy == 0)
        {
            if (-idum < 1) idum = 1;
            else idum = -idum;
            for (var j = Ntab + 7; j >= 0; j--)
            {
                var k = idum / Iq;
                idum = Ia * (idum - k * Iq) - Ir * k;
                if (idum < 0) idum += Im;
                if (j < Ntab) iv[j] = idum;
            }
            iy = iv[0];
        }

        var nextK = idum / Iq;
        idum = Ia * (idum - nextK * Iq) - Ir * nextK;
        if (idum < 0) idum += Im;
        var index = iy / Ndiv;
        if ((uint)index >= Ntab) index &= Ntab - 1;
        iy = iv[index];
        iv[index] = idum;
        return iy;
    }
}
