#include "../src/ardrone_config.h"
#include "../src/ardrone_parser.h"
#include <stdio.h>

int main(int argc, char* argv[]){
    if (argc < 2){
        printf("Usage: %s <path to config file>\n", argv[0]);
        return 1;
    }

    ArDroneConfig config;
    ArDroneParser parser;

    parser.readFile(argv[1], &config);
    config.printParameters();

    return 0;
}
