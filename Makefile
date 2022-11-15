all:switcher_danfysik

switcher_danfysik: switcher_danfysik.c
	$(CC) -o switcher_danfysik -Wall -g switcher_danfysik.c -I. -lXm

clean:
	$(RM) switcher_danfysik
