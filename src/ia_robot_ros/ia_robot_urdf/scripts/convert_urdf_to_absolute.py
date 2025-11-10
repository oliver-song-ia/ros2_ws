
import os
import sys

def main():
    input_name = sys.argv[1] if len(sys.argv) == 2 else 'ia_robot.urdf'
    base, ext = os.path.splitext(input_name)
    output_name = f'{base}.absolute{ext}'

    urdf_file = os.path.join(os.path.dirname(__file__), f'../urdf/{input_name}')
    new_urdf_file = os.path.join(os.path.dirname(__file__), f'../urdf/{output_name}')

    # Get the absolute path to the "ia_robot_urdf" folder (three levels up from this script)
    current_folder = os.path.abspath(os.path.dirname(__file__))
    current_folder = os.path.dirname(current_folder)
    current_folder = os.path.dirname(current_folder)

    old_str = "package:/"
    new_str = current_folder

    # Read the file
    with open(urdf_file, "r") as file:
        content = file.read()

    # Replace all occurrences
    content = content.replace(old_str, new_str)

    # Write back to the file
    with open(new_urdf_file, "w") as file:
        file.write(content)

    print(f"Replaced all '{old_str}' with '{new_str}' in {urdf_file}")
    print(f"Created new URDF file at {new_urdf_file}")

if __name__ == "__main__":
    main()