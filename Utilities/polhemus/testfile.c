#include <stdio.h>
#include <stdlib.h>

#include "polhemus.h"

int main(int argc, char *argv[])
{
  char text[256];
  char *cp = text;
  double x, y;

  if (argc < 2) {
    return 1;
  }

  strncpy(text,argv[1],256);
  x = phAsciiExToFloat(&cp);
  y = phAsciiExToFloat(&cp);

  printf("\'%s\' \'%g\' \'%g\'\n", text, x, y);

  return 0;
}
