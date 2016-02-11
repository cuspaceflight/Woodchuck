# Define project name here
PROJECT = woodchuck

IDIR =../libopencm3/include
HDIR = ./
CC=arm-none-eabi-gcc

CFLAGS=-I$(IDIR) -DSTM32F0 
ODIR=./

_DEPS = *.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))
HDEPS = $(patsubst %,$(HDIR)/%,$(_DEPS))

_OBJ = radio.o 
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.c $(DEPS) $(HDEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

all: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 