all: win linux mac

win:
	g++ -v -shared -o simExtLibTest.dll -I $(CURDIR)/include simExtLibTest.c -lws2_32 -fPIC -g
	#x86_64-w64-mingw32-g++-win32 -shared -o simExtLibTest.dll simExtLibTest.c include/simLib.h include/stack/stackMap.h include/stack/stackArray.h -fPIC

linux:
	g++ -shared -o libSimExtLibTest.so simExtLibTest.c -I include -fPIC -g

mac:
	g++ -shared -o libSimExtLibTest.dylib simExtLibTest.c -I include -fPIC -g