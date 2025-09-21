# heat index lookup table calculator
# from 10C to 50C
# from 0% to 100% humidity
# step 1C, 1% humidity

temps = list(range(10, 51))
humidities = list(range(0, 101))

def heat_index(temp_c, humidity):
    c1 = -8.78469475556
    c2 = 1.61139411
    c3 = 2.33854883889
    c4 = -0.14611605
    c5 = -0.012308094
    c6 = -0.0164248277778
    c7 = 0.002211732
    c8 = 0.00072546
    c9 = -0.000003582

    hi = (c1 + (c2 * temp_c) + (c3 * humidity) + (c4 * temp_c * humidity) +
          (c5 * temp_c ** 2) + (c6 * humidity ** 2) +
          (c7 * temp_c ** 2 * humidity) + (c8 * temp_c * humidity ** 2) +
          (c9 * temp_c ** 2 * humidity ** 2))
    return round(hi, 1)

# create lookup table
lookup_table = []
for temp in temps:
    row = []
    for humidity in humidities:
        hi = heat_index(temp, humidity)
        row.append(hi)
    lookup_table.append(row)

# pretty print table in python source format
print("heat_index_lut = [")
for row in lookup_table:
    print("    " + str(row) + ",")
print("]")


