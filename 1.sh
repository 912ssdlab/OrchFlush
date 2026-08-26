#!/usr/bin/bash
limitsFile=/etc/security/limits.conf
if [ "unlimited" != "`egrep "^*.*soft.*core.*" $limitsFile | awk -F' ' '{print $4}'`" ]; then
    echo "set core size to unlimited"
    sed  -i  's/^#\*\(.*soft\)\(.*core\)\(.*\)0/* \1\2\3unlimited/' $limitsFile
fi

