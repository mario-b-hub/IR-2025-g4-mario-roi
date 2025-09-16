#!/bin/bash

centrar_figlet() {
    figlet "$*" | awk -v term_width="$(tput cols)" '{ 
        line_length = length($0); 
        padding = int((term_width - line_length) / 2); 
        printf "%"padding"s%s\n", "", $0 
    }'
}

centrar() {
    local text="$1"
    local width=$(tput cols)
    local padding=$(( (width - ${#text}) / 2 ))
    printf "%*s\n" $((padding + ${#text})) "$text"
}

sangrar() {
    local espacios="${1}"
    shift

    if ! [[ "$espacios" =~ ^[0-9]+$ ]]; then
        echo "Error: el primer argumento debe ser un número (cantidad de espacios de sangrado)." >&2
        return 1
    fi

    local comando="$@"

    # Ejecuta el comando, agrega espacios al inicio de cada línea
    "$@" | sed "s/^/$(printf ' %.0s' $(seq 1 $espacios))/"
}

echo -e "\e[33m$(centrar_figlet "Practica")\e[0m"
echo -e "\e[33m$(centrar_figlet "Uno")\e[0m"

centrar " "
echo -e "\e[33m$(centrar "Welcome to the linux terminal. What do you want to do?")\e[0m"

echo -e "\e[33m        | MEMBERS |\e[0m"
echo "	
	 -----------------------
	| Roi Seoane Punti	|
	|			|
	| Mario Badia Portillo	|
	 -----------------------
"	
echo -e "\e[33m        | SCRIPT PATH |\e[0m"
echo " "
sangrar 8 realpath welcome.sh

echo " "

echo -e "\e[33m        | FILES AND PERMISIONS |\e[0m"
echo " "
sangrar 8 ls -l




echo " "
