min_val = -22.5
max_val = 22.5
num_entries = 128
def generate_cpp_array(num_entries, min_val, max_val):
    step = (max_val - min_val) / (num_entries-1)
    array = [min_val + step * i for i in range(num_entries)]
    new_array = [0] * num_entries
    middle = round(num_entries*(abs(min_val)/(abs(min_val) + abs(max_val))))
    for i in range(round(num_entries/2)):
        j = num_entries - i - 1
        k = round(middle) - i - 1
        new_array[i*2] = array[j]
        new_array[i*2 + 1] = array[k]


    array_str = "{" + ", ".join([f"{val:.1f}f" for val in new_array]) + "};"
    return array_str
 
def generate_array(num_entries, min_val, max_val):
    step = (max_val - min_val) / (num_entries-1)
    array = [min_val + step * i for i in range(num_entries)]
    array_str = "{" + ", ".join([f"{val:.1f}f" for val in array]) + "};"
    return array_str

cpp_array = generate_array(num_entries, min_val, max_val)
print(cpp_array)
