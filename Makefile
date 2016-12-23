rglob=$(wildcard \
	$(1)/$(2) \
	$(1)/*/$(2) \
	$(1)/*/*/$(2) \
	$(1)/*/*/*/$(2) \
)

cxx := g++
cxxflags := $(strip \
	-Wall -Wreturn-type -pedantic \
	-std=c++11 -g -O3 \
	-Iinclude \
	-msse -msse2 \
)
ldlibs := -lSDL2 -lSDL2_ttf
cxxfiles := $(call rglob,src,*.cpp)
ofiles := $(patsubst src/%.cpp,build/%.o,$(cxxfiles))
exe := bin/test_verlet

.PHONY: all clean re mrproper
all: $(exe)
clean:
	rm -rf build
re: clean all
mrproper: re

build/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(cxx) $(cxxflags) -c $< -o $@

$(exe): $(ofiles)
	@mkdir -p $(@D)
	$(cxx) $(cxxflags) $^ -o $@ $(ldlibs)

