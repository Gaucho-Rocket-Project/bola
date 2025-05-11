import polars as pl
import numpy as np

def to_cpp_array(name, arr, dtype="double"):
    arr_str = ", ".join(map(str, arr))
    return f"{dtype} {name}[] = {{ {arr_str} }};"

def main():
    df = pl.read_csv("/ESP32/testFlash/data/servo_test_data_-_strong_step_test_force.csv") # Use your actual file path (FULL PATH REQUIRED!)
    df = df.drop_nulls()

    header = "#pragma once\n\n"
    arrays = []
    for col in df.columns:
        arr = df[col].to_numpy()
        dtype = "float" if col == "Time" else "double"
        arrays.append(to_cpp_array(col.lower().replace(" ", "_"), arr, dtype))
    with open("strong_step_test_force.h", "w") as f:
        f.write(header)
        for arr in arrays:
            f.write(arr + "\n")

if __name__ == "__main__":
    main()