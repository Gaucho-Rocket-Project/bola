import polars as pl
import numpy as np

def to_cpp_array(name, arr, dtype="double"):
    # .31f is the max precision for float in ESP32 (32-bit limit)
    if name == "time":
        arr_str = ", ".join(f"{x:.2f}" for x in arr)
        dtype = "float"
    else:
        arr_str = ", ".join(f"{x:.31f}" for x in arr)
    return f"{dtype} {name}[] = {{ {arr_str} }};"

def main():
    df = pl.read_csv(
        "/home/cheng/Downloads/Rocket/ESP32/testFlash/data/strong_step_test_force.csv",
        schema_overrides={
            "Time": pl.Float64,
            "Roll": pl.Float64,
            "Pitch": pl.Float64,
            "Roll Correction": pl.Float64,
            "Pitch Correction": pl.Float64,
            "Height": pl.Float64,
        }
    )
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