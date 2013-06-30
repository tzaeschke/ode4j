mkdir $1
cd $1
tar cvf ode4j-${1}.tar ode4j.jar
tar cvf ode4j-cpp-${1}.tar ode4j-cpp.jar
#tar cvf ode4j-src-${1}.tar ~/projects/cpp4j/src ~/projects/ode4j-core/src ~/projects/ode4j-cpp/src ~/projects/ode4j-sdk/src
tar -C ~/projects/cpp4j --exclude "*.svn*" -cvf ode4j-src-${1}.tar src
tar -C ~/projects/ode4j-core --exclude "*.svn*" -rvf ode4j-src-${1}.tar src
tar -C ~/projects/ode4j-cpp --exclude "*.svn*" -rvf ode4j-src-${1}.tar src
tar -C ~/projects/ode4j-sdk --exclude "*.svn*" -rvf ode4j-src-${1}.tar src
#tar cvf ode4j-doc-${1}.tar ~/projects/ode4j-sdk/doc
tar -C ~/projects/ode4j-sdk --exclude "*.svn*" -cvf ode4j-doc-${1}.tar doc
gzip ode4j-${1}.tar
gzip ode4j-cpp-${1}.tar
gzip ode4j-src-${1}.tar
gzip ode4j-doc-${1}.tar
cd ..

