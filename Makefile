rglob=$(wildcard \
	$(1)/$(2) \
	$(1)/*/$(2) \
	$(1)/*/*/$(2) \
	$(1)/*/*/*/$(2) \
	$(1)/*/*/*/*/$(2) \
	$(1)/*/*/*/*/*/$(2) \
	$(1)/*/*/*/*/*/*/$(2) \
	$(1)/*/*/*/*/*/*/*/$(2) \
	$(1)/*/*/*/*/*/*/*/*/$(2) \
	$(1)/*/*/*/*/*/*/*/*/*/$(2) \
	$(1)/*/*/*/*/*/*/*/*/*/*/$(2) \
	$(1)/*/*/*/*/*/*/*/*/*/*/*/$(2) \
	$(1)/*/*/*/*/*/*/*/*/*/*/*/*/$(2) \
)

cxx := g++
cxxflags := -Wall -pedantic -std=c++11 -g
ldlibs := -lSDL2
cxxfiles := $(call rglob,src,*.cpp)
ofiles := $(patsubst src/%.cpp,build/%.o,$(cxxfiles))

.PHONY: all clean re mrproper
all: bin/toast
clean:
	rm -rf build
re: clean all
mrproper: re

build/%.o: src/%.o
	@mkdir -p $(@D)
	$(cxx) $(cxxflags) -c $< -o $@

bin/toast: $(ofiles)
	@mkdir -p $(@D)
	$(cxx) $(cxxflags) $^ -o $@ $(ldlibs)

