import pynmea2
from prompt_toolkit import prompt
from colorama import init, Fore, Style

# Initialize colorama
init()

def parse_nmea(sentence):
    try:
        msg = pynmea2.parse(sentence)
        return msg
    except pynmea2.ParseError:
        return None

def format_nmea(msg):
    if msg is None:
        return f"{Fore.RED}Invalid NMEA sentence{Style.RESET_ALL}"
    
    formatted = f"{Fore.CYAN}Sentence Type: {msg.sentence_type}{Style.RESET_ALL}\n"
    for field in msg.fields:
        if hasattr(msg, field[1]):
            value = getattr(msg, field[1])
            formatted += f"{Fore.YELLOW}{field[1]}{Style.RESET_ALL}: {value}\n"
    return formatted

def main():
    print(f"{Fore.GREEN}Welcome to the NMEA Parser TUI!{Style.RESET_ALL}")
    print("Paste your NMEA sentences (one per line). Enter a blank line to finish input.")
    print("Type 'quit' to exit the program.")

    while True:
        sentences = []
        while True:
            line = prompt("NMEA > ").strip()
            if line.lower() == 'quit':
                print(f"{Fore.GREEN}Goodbye!{Style.RESET_ALL}")
                return
            if not line:
                break
            sentences.append(line)
        
        if not sentences:
            continue

        print(f"\n{Fore.BLUE}Parsed NMEA data:{Style.RESET_ALL}")
        for sentence in sentences:
            msg = parse_nmea(sentence)
            formatted = format_nmea(msg)
            print(f"\n{Fore.MAGENTA}{sentence}{Style.RESET_ALL}")
            print(formatted)
        
        print("=" * 50)

if __name__ == "__main__":
    main()