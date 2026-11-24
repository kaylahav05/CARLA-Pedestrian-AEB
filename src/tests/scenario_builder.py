
from tests.scenarioconfig import SCENARIOS, FIXED
from pprint import pprint

def build_test_cases(scenarios, fixed, only=None):
    test_cases = []

    # Loop through each scenario
    for scenario_name, scenario in scenarios.items():

        if only is not None and scenario_name != only:
            continue

        # Loop through each variation
        for variation_name, variation in scenario["variations"].items():

            # Exclude lists used to generate combinations
            variation_data = {
                key: value
                for key, value in variation.items()
                if key not in ("vehicle_speeds", "conditions")
            }

            # Loop through each condition
            for condition in variation["conditions"]:

                # Loop through each vehicle speed
                for speed in variation["vehicle_speeds"]:

                    # Build individual test case
                    test_case = {
                        **fixed,
                        **variation_data,
                        **condition,
                        "scenario": scenario_name,
                        "variation": variation_name,
                        "vehicle_speed": speed,
                    }

                    # Calculate initial headway (meters)
                    vehicle_speed_ms = speed / 3.6

                    # Pedestrian moves perpendicular to vehicle unless is along path
                    is_along = variation["direction"] == "right-forward"
                    pedestrian_longitudinal_speed = (test_case["pedestrian_speed"] / 3.6) if is_along else 0.0      

                    test_case["headway_m"] = (
                        test_case["initial_ttc"] *
                        (vehicle_speed_ms - pedestrian_longitudinal_speed)
                    )

                    # Generate unique test ID
                    test_case["test_id"] = (
                        f"{variation['section']}_"
                        f"{condition['lighting']}_"
                        f"{speed}kmh_"
                        f"{int(condition['overlap'] * 100)}pct"
                    )

                    # Add test case to list
                    test_cases.append(test_case)

    return test_cases

