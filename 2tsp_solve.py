import numpy as np
import math
import time

#IMPLEMENTED BY: MUHAMMET GÖRKEM GÜLMEMİŞ, AYŞENUR TÜFEKCİ, MERT EFE KARAKÖSE

# In this step, we are checking if a city has been visited or not.
def is_city_visited(city, visited_cities):
    return visited_cities[city] == 1

# Here, we find the next city to visit based on the A* algorithm.
# We calculate the f-value for each unvisited city and select the city with the minimum f-value.
def find_min_distance_city(last_city, visited_cities, distance_matrix, heuristic_values, total_cities):
    min_distance = float('inf')
    min_distance_city = -1
    for i in range(total_cities):
        if i != last_city and not is_city_visited(i, visited_cities):
            distance = distance_matrix[last_city, i]
            h_value = heuristic_values[i]
            f_value = distance + h_value
            if f_value < min_distance:
                min_distance = f_value
                min_distance_city = i
    return min_distance_city

# This function performs the A* algorithm to determine the route for one of the salesmen.
# Starting from the start_city, it visits the specified number of cities.
def a_star_algorithm(start_city, visited_cities, visited_cities_flags, distance_matrix, heuristic_values, salesman_city_count, total_cities):
    visited_count = 1
    current_city = start_city
    visited_cities[0] = current_city
    visited_cities_flags[current_city] = 1
    while visited_count < salesman_city_count:
        next_city = find_min_distance_city(current_city, visited_cities_flags, distance_matrix, heuristic_values, total_cities)
        visited_cities[visited_count] = next_city
        visited_cities_flags[next_city] = 1
        visited_count += 1
        current_city = next_city

def main():
    start_time = time.time()

    # Define the list of input and output files.
    input_files = ['test-input-1.txt', 'test-input-2.txt', 'test-input-3.txt', 'test-input-4.txt']
    output_files = ['test-output-1.txt', 'test-output-2.txt', 'test-output-3.txt', 'test-output-4.txt']
    max_rows = 50000

    for input_file, output_file in zip(input_files, output_files):
        # We initialize the array to store city coordinates.
        city_coordinates = np.zeros((max_rows, 3), dtype=float)
        total_cities = 0

        # Here, we read city coordinates from the input file.
        with open(input_file, 'r') as file:
            for line in file:
                if total_cities < max_rows:
                    city_data = list(map(float, line.split()))
                    if len(city_data) == 3:
                        city_coordinates[total_cities] = city_data
                        total_cities += 1
                    else:
                        print(f"Warning: Invalid city data format in line: {line.strip()}")

        # We calculate the distance matrix for all cities.
        distance_matrix = np.zeros((total_cities, total_cities), dtype=int)
        for i in range(total_cities):
            for j in range(total_cities):
                dx = city_coordinates[i, 1] - city_coordinates[j, 1]
                dy = city_coordinates[i, 2] - city_coordinates[j, 2]
                distance = int(round(math.sqrt(dx ** 2 + dy ** 2)))
                distance_matrix[i, j] = distance

        # We calculate total distances from each city to all other cities.
        city_total_distances = np.sum(distance_matrix, axis=1)

        # In this step, we find the city with the minimum total distance to start the algorithm.
        start_city = np.argmin(city_total_distances)
        salesman_city_count_1 = math.ceil(total_cities / 2)
        salesman_city_count_2 = total_cities - salesman_city_count_1

        # We initialize arrays for visited cities and flags for both salesmen.
        visited_cities_flags_1 = np.zeros(total_cities, dtype=int)
        visited_cities_flags_2 = np.zeros(total_cities, dtype=int)
        salesman_routes = [np.full(salesman_city_count_1, -1, dtype=int), np.full(salesman_city_count_2, -1, dtype=int)]
        heuristic_values = np.full(total_cities, float('inf'), dtype=float)

        # Here, we calculate heuristic values for the A* algorithm.
        for i in range(total_cities):
            for j in range(total_cities):
                if distance_matrix[j, i] < heuristic_values[i] and i != j:
                    heuristic_values[i] = distance_matrix[j, i]

        # Run the A* algorithm for the first salesman.
        a_star_algorithm(start_city, salesman_routes[0], visited_cities_flags_1, distance_matrix, heuristic_values, salesman_city_count_1, total_cities)
        
        # Mark the cities visited by the first salesman.
        for city in salesman_routes[0]:
            visited_cities_flags_2[city] = 1

        # Run the A* algorithm for the second salesman on the remaining cities.
        remaining_cities = [i for i in range(total_cities) if visited_cities_flags_2[i] == 0]
        if remaining_cities:
            a_star_algorithm(remaining_cities[0], salesman_routes[1], visited_cities_flags_2, distance_matrix, heuristic_values, salesman_city_count_2, total_cities)

        # Calculate the total distances for both salesmen.
        total_distance1 = sum(distance_matrix[salesman_routes[0][i], salesman_routes[0][i + 1]] for i in range(salesman_city_count_1 - 1))
        total_distance1 += distance_matrix[salesman_routes[0][salesman_city_count_1 - 1], salesman_routes[0][0]]

        total_distance2 = sum(distance_matrix[salesman_routes[1][i], salesman_routes[1][i + 1]] for i in range(salesman_city_count_2 - 1))
        total_distance2 += distance_matrix[salesman_routes[1][salesman_city_count_2 - 1], salesman_routes[1][0]]

        # Write the results to the output file.
        with open(output_file, 'w') as file:
            file.write(f"{total_distance1 + total_distance2}\n")
            file.write(f"{total_distance1} {salesman_city_count_1}\n")
            for city in salesman_routes[0]:
                file.write(f"{city}\n")
            file.write("\n")
            file.write(f"{total_distance2} {salesman_city_count_2}\n")
            for city in salesman_routes[1]:
                file.write(f"{city}\n")

    # Calculate and print the execution time in seconds.
    elapsed_time = time.time() - start_time
    print(f"Execution time: {elapsed_time} seconds")

if __name__ == "__main__":
    main()
