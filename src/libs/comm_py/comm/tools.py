

def server2int(server):
    serverl = server.lower()
    if serverl == "ice" or serverl == "1":
        return 1
    elif serverl == "ros" or serverl == "2":
        return 2
    else : return 0