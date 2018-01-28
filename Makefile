all: hello.so

hello.so: hello.cpp IKSolver.cpp
	g++ -shared -O3 -o $@ -lpython $^

clean:
	-rm *.so *.dylib
