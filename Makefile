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
cxxflags := -Wall -Wreturn-type -pedantic -std=c++11 -g -Iinclude -Iinclude/contrib
ldlibs := -lSDL2 -lSDL2_ttf -lfreetype
cxxfiles := $(call rglob,src,*.cpp)
ofiles := $(patsubst src/%.cpp,build/%.o,$(cxxfiles))

.PHONY: all clean re mrproper
all: bin/toast
clean:
	rm -rf build
re: clean all
mrproper: re

build/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(cxx) $(cxxflags) -c $< -o $@

bin/toast: $(ofiles)
	@mkdir -p $(@D)
	$(cxx) $(cxxflags) $^ -o $@ $(ldlibs)

