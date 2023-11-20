def main():
    # Prompt the user for the file name
    file_name = input("Enter the name of the text file: ")

    try:
        # Open the file for reading
        with open(file_name, 'r') as file:
            total_sum = 0

            # Read each line, convert it to an integer, and add it to the total sum
            for line in file:
                try:
                    number = float(line)
                    total_sum += number
                except ValueError:
                    print(f"Skipping non-integer line: {line}")

            # Print the total sum
            print(f"Total Sum: {total_sum}")

    except FileNotFoundError:
        print(f"File '{file_name}' not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()