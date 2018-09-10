#include <stdio.h>

int main(int argc, char const *argv[])
{
	int a = -1;
	unsigned char b[] = {0xff, 0xff, 0xff, 0xff};
 	unsigned short c1, c2;
	c1 = b[0] << 8 | b[1];
	c2 = b[2] << 8 | b[3];
	int d = c1 << 16 | c2;
	printf("a:%x, c1:%x, c2:%x, d:%x %d\n", a, c1, c2, d, d);


	short f = 0x81a7;
	printf("f:%x, f:%x, %x, %x\n", f, f, (unsigned short)f, (short)(unsigned short)f);
	return 0;
}
