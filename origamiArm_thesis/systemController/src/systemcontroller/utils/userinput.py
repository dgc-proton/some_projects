def get_int(*, prompt: str, num_max: int | None = None, num_min: int | None = None) -> int:
    """Gets an integer from the user."""
    while True:
        try:
            num = int(input(f"{prompt}: "))
        except ValueError:
            print("Error converting the input to integer, please re-enter.")
            continue
        if num_max and (num > num_max):
            print(f"Number entered exceeds max number allowed ({num_max}), please re-enter.")
            continue
        if num_min and (num < num_min):
            print(f"Number entered is less than min number allowed ({num_min}), please re-enter.")
            continue
        break
    return num
