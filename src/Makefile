IDIR = ../includes
#
CC = g++
# CFLAGS = $(IDIR:%=-I%)
CFLAGS = -I$(IDIR)

ODIR = obj

LDIR = -I/home/julien/Documents/eigen/ -L/usr/local/lib/
LIBS = -ltinyxml2 -lglpk -lm

_DEPS = robot.h contactPoints.h vertex.h edge.h face.h
DEPS = $(_DEPS:%=$(IDIR)/%)

_OBJ = main.o robot.o contactPoints.o vertex.o edge.o face.o
OBJ = $(_OBJ:%=$(ODIR)/%)


$(ODIR)/%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(LDIR) $(LIBS)

main: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LDIR) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(IDIR)/*~
