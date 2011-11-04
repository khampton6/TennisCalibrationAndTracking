export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig
export LDLIBRARY_PATH=/usr

CXX = g++
CXXFLAGS = -g -bind_at_load `pkg-config --cflags opencv`
LFLAGS = `pkg-config --libs opencv`


SRC = main.cpp LinePrediction.cpp ModelFitting.cpp

OBJS = $(SRC:.cpp=.o)

demo: $(OBJS)                                                              
	$(CXX) $(CXXFLAGS) $(LFLAGS) $(OBJS) -o $@
all: demo

clean:
	rm -f $(OBJS) demo
