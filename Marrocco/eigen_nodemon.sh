location="eigen.cpp";

clear; printf "\033[1;33mWatching for saves in $location\n\n\033[0;37m";
while inotifywait -q -q --event modify $location; do { clear; printf "\033[1;33mWatching for saves in $location\n\n\033[0;37m"; g++ eigen.cpp -o eigen; ./eigen; }; done;
