#!/bin/csh -f

foreach arg ( $* )
    /bin/cp $arg $arg.new
    chmod u+w $arg.new
    /bin/sed -f ./insertlic.sed $arg >! $arg.new
    /bin/mv -f $arg.new $arg
end
