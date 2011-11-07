export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig
export LDLIBRARY_PATH=/usr

CXX = g++
CXXFLAGS = -g -bind_at_load `pkg-config --cflags opencv`
LFLAGS = `pkg-config --libs opencv`


SRC = main.cpp LinePrediction.cpp ModelFitting.cpp Player.cpp
SRC2 = Player.cpp

OBJS = $(SRC:.cpp=.o)
PlayerOBJS = $(SRC2:.cpp=.o)

all: demo

demo: $(OBJS)                                                              
	$(CXX) $(CXXFLAGS) $(LFLAGS) $(OBJS) -o $@

player: $(PlayerOBJS)
	$(CXX) $(CXXFLAGS) $(LFLAGS) $(PlayerOBJS) -o $@

clean:
	rm -f $(OBJS) demo $(PlayerOBJS) player
