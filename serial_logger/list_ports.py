from serial.tools import list_ports

def main():
    ports = list_ports.comports()
    if not ports:
        print("No serial ports found.")
    else:
        print("Available serial ports:")
        for p in ports:
            print(f"{p.device} : {p.description}")

if __name__ == "__main__":
    main()
