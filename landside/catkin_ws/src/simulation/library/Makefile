all: win linux mac

win:
	x86_64-w64-mingw32-g++-win32 -shared -o simExtLibTest.dll -static -static-libgcc -static-libstdc++ -I include -I include/stack common/*.cpp common/stack/*.cpp simExtLibTest.c -fPIC -g -lshlwapi -lwinmm -lws2_32
#	x86_64-w64-mingw32-g++-win32 -shared -c simExtLibTest.c -I include -fPIC -g
#	x86_64-w64-mingw32-g++-win32 -shared -o simExtLibTest.dll simExtLibTest.o -I include -fPIC -g

linux:
	g++ -shared -o libSimExtLibTest.so -I include -I include/stack common/*.cpp common/stack/*.cpp simExtLibTest.c -fPIC -g


mac:
	g++ -shared -o libSimExtLibTest.dylib -I include -I include/stack common/*.cpp common/stack/*.cpp simExtLibTest.c -fPIC -g

clean:
	rm simExtLibTest.dll libSimExtLibTest.*

me: win
	cp simExtLibTest.dll "/mnt/c/Program Files/CoppeliaRobotics/CoppeliaSimEdu"