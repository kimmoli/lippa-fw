#include "hal.h"
#include "helpers.h"

BaseSequentialStream* consoleStream;

void dump(const char* data, int len)
{
    int n = 0;
    do
    {
        int i = MIN(len-n, 16);
        int j = 0;

        PRINT("%04x ", n);

        for(j=n;j<(n+i);j++)
            PRINT(" %02X", data[j]);

        for(j=0;j<(16-i)+1;j++)
            PRINT("   ");

        for(j=n;j<(n+i);j++)
            PRINT("%c", ISPRINT(data[j]));

        PRINT("\n\r");
        n += i;
    }
    while (n < len);
}
