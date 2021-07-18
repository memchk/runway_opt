import json
import itertools
import pandas as pd
from ortools.sat.python import cp_model
import plotly.graph_objects as go # type: ignore

CENTERLINE_FILE = 'center.geojson'
EAST_FILE = 'east.geojson'
WEST_FILE = 'west.geojson'

SYMMETRIC_GRADE = False

# In milimeters
MAX_HEIGHT = 1000 * 1000

# Bounds on Grade in hundredths of a percent
MIN_GRADE = int(1.25 * 100)
MAX_GRADE = int(3 * 100)
MAX_SLOPE = 2

# In milimeters
CENTER_TO_SIDE_LENGTH = int(13.41 * 1000)
POINT_TO_POINT_LENGTH = int(1 * 1000)

# Edge Deviations
MAX_EDGE_DEVIATION = 275
WEST_DEVIATION_LIMITS = [
# Dean's Taxiway,
    (153, 175, 175),
# Brad Taxiway
    (298, 328, 127),
]


SMOOTHNESS_SCALER = 10000**2
DER_BOUND = SMOOTHNESS_SCALER * MAX_HEIGHT
MAX_UNSMOOTH = int(0.007 * SMOOTHNESS_SCALER / 100)

NO_BACKBONE_INCREASE_RANGE = (18, 264)



NORTH_BOUNDARY = 439
UNSMOOTH_OVEREXTEND = 2


def get_coords(file):
    with open(file) as f:
        json_file = json.load(f)
        coords = json_file['features'][0]['geometry']['coordinates'][0]
        return [[int(x[0] * 1000), int(x[1] * 1000), int(x[2] * 1000)] for x in coords]


def window(arr, k):
    for i in range(len(arr)-k+1):
        yield arr[i:i+k]


def grade_constraint(model: cp_model.CpModel, center_var, edge_var):
    center_name = center_var.Name()
    grade_var = model.NewIntVar(MIN_GRADE, MAX_GRADE, "grade_" + center_name)
    num_var = model.NewIntVar(0, MAX_HEIGHT * 10000, "num_" + center_name)

    model.Add(num_var == 10000 * (center_var - edge_var))
    model.AddDivisionEquality(grade_var, num_var, CENTER_TO_SIDE_LENGTH)

    return grade_var


def smoothness_constraint(model: cp_model.CpModel, vars):
    der_vars = []
    for i,w in enumerate(window(vars, 3)):
        num_var = model.NewIntVar(-DER_BOUND, DER_BOUND, "der_num_" + str(i))
        der_var = model.NewIntVar(-MAX_UNSMOOTH, MAX_UNSMOOTH, "der_" + str(i))
        model.Add(num_var == SMOOTHNESS_SCALER*(w[2] - 2*w[1] + w[0]))
        model.AddDivisionEquality(der_var, num_var, POINT_TO_POINT_LENGTH**2)
        der_vars.append(der_var)
    
    return der_vars


def matching_constraint(model: cp_model.CpModel, abs_error_vars, limit):
    for e in abs_error_vars:
        model.Add(e <= limit)


def add_dirt_constraint(model: cp_model.CpModel, runway_vars, runway_coords, min_add):
    for v,c in zip(runway_vars, runway_coords):
        model.Add(v[2] - c[2][2] >= min_add)
        model.Add(v[1] - c[1][2] >= min_add)
        model.Add(v[0] - c[0][2] >= min_add)


def change_goal(model: cp_model.CpModel, abs_error_terms_goal):
    error_eq = sum(abs_error_terms_goal)
    model.Add(error_eq >= 0)
    return error_eq


def derivative_goal(model: cp_model.CpModel, east_der_vars, center_der_vars, west_der_vars):
    der_vars = []
    der_vars += east_der_vars
    der_vars += center_der_vars
    der_vars += west_der_vars

    abs_der_vars = []
    for i, d in enumerate(der_vars):
        abs_der_err = model.NewIntVar(0, DER_BOUND, "abs_der_error_" + str(i))
        model.AddAbsEquality(abs_der_err, d)
        abs_der_vars.append(abs_der_err)

    der_err = model.NewIntVar(0, len(der_vars)*DER_BOUND, "der_error")
    model.Add(der_err == sum(abs_der_vars))
    return der_err


def get_dataframe(solver, runway_vars, runway_coords):
    raw_data = []
    for i, (v,r) in enumerate(zip(runway_vars, runway_coords)):
        raw_data.append([
            (i * POINT_TO_POINT_LENGTH) / 1000.0,
            solver.Value(v[0]) / 1000.0,
            solver.Value(v[1]) / 1000.0,
            solver.Value(v[2]) / 1000.0,
            (solver.Value(v[0]) - r[0][2]) / 1000.0,
            (solver.Value(v[1]) - r[1][2]) / 1000.0,
            (solver.Value(v[2]) - r[2][2]) / 1000.0,
        ])
    
    return pd.DataFrame(raw_data, columns=["x", 
        "west_height", "center_height", "east_height",
        "west_diff", "center_diff", "east_diff"])


def plot_dataframe(df):
    fig = go.Figure() # type: ignore

    fig.add_trace(go.Scatter( # type: ignore
        x=df['x'],
        y=df['west_height'],
        legendgroup="abs",
        legendgrouptitle_text="Absolute Heights",
        mode='lines',
        name="West Height (m)"
    ))

    fig.add_trace(go.Scatter( # type: ignore
        x=df['x'],
        y=df['center_height'],
        legendgroup="abs",
        mode='lines',
        name="Center Height (m)"
    ))

    fig.add_trace(go.Scatter( # type: ignore
        x=df['x'],
        y=df['east_height'],
        legendgroup="abs",
        mode='lines',
        name="East Height (m)"
    ))

    fig.add_trace(go.Scatter( # type: ignore
        x=df['x'],
        y=df['west_diff'],
        legendgroup="diff",
        legendgrouptitle_text="Differences",
        mode='lines',
        name="West Difference (m)"
    ))

    fig.add_trace(go.Scatter( # type: ignore
        x=df['x'],
        y=df['center_diff'],
        mode='lines',
        legendgroup="diff",
        name="Center Difference (m)"
    ))

    fig.add_trace(go.Scatter( # type: ignore
        x=df['x'],
        y=df['east_diff'],
        mode='lines',
        legendgroup="diff",
        name="East Difference (m)"
    ))

    fig.update_layout(showlegend=True)
    fig.show()


def main():
    center = get_coords(CENTERLINE_FILE)
    west = get_coords(WEST_FILE)
    east = get_coords(EAST_FILE)

    if(len(center) != len(west) and len(center) != len(east)):
        print("ERROR: Each line must have the same number of points!")
        return
    
    num_points = len(center)

    runway_coords = list(zip(west, center, east))
    all_runway_coords = list(itertools.chain.from_iterable(runway_coords))

    model = cp_model.CpModel()
    runway_vars = [(
        model.NewIntVar(0, MAX_HEIGHT, 'west_' + str(x)),
        model.NewIntVar(0, MAX_HEIGHT, 'center_' + str(x)),
        model.NewIntVar(0, MAX_HEIGHT, 'east_' + str(x))
    ) for x in range(num_points)]

    abs_error_vars = [(
        model.NewIntVar(0, MAX_HEIGHT, "abs_error_west_" + str(x)),
        model.NewIntVar(0, MAX_HEIGHT, "abs_error_center_" + str(x)),
        model.NewIntVar(0, MAX_HEIGHT, "abs_error_east_" + str(x))
    ) for x in range(num_points)]

    all_runway_vars = list(itertools.chain.from_iterable(runway_vars))
    all_abs_error_vars = list(itertools.chain.from_iterable(abs_error_vars))
    

    west_der_vars = smoothness_constraint(model, [x[0] for x in smoothness_domain])
    center_der_vars = smoothness_constraint(model, [x[1] for x in smoothness_domain])
    east_der_vars = smoothness_constraint(model, [x[2] for x in smoothness_domain])

    # Setup Grade limits
    cw_grade_vars = []
    ce_grade_vars = []

    for v in runway_vars[:NORTH_BOUNDARY]:
        # Center to West
        cw_grade = grade_constraint(model, v[1], v[0])
        # Center to East
        ce_grade = grade_constraint(model, v[1], v[2])

        cw_grade_vars.append(cw_grade)
        ce_grade_vars.append(ce_grade)

        if SYMMETRIC_GRADE:
            model.Add(cw_grade == ce_grade)

    # Constrain Centerline to only cut past Brad's House (going south) which is i <= 27
    # Also, Constrain edges to a maximum deviation of MAX_EDGE_DEVIATION
    for i, (v,e,r) in enumerate(zip(runway_vars[:NORTH_BOUNDARY], abs_error_vars, runway_coords)):
        if NO_BACKBONE_INCREASE_RANGE[0] <= i <= NO_BACKBONE_INCREASE_RANGE[1]:
            model.Add(r[1][2] - v[1] >= 0)
        model.Add(e[0] <= MAX_EDGE_DEVIATION)
        model.Add(e[2] <= MAX_EDGE_DEVIATION)

    # Gordo-Cooper Taxiway Match (East)
    matching_constraint(model, [x[2] for x in abs_error_vars[304:321]], 100)

    for (l, u, m) in WEST_DEVIATION_LIMITS:
        matching_constraint(model, [x[0] for x in abs_error_vars[l:u + 1]], m)

    # No Touch past NORTH_BOUNDARY
    matching_constraint(model, [x[0] for x in abs_error_vars[NORTH_BOUNDARY:]], 0)
    matching_constraint(model, [x[1] for x in abs_error_vars[NORTH_BOUNDARY:]], 0)
    matching_constraint(model, [x[2] for x in abs_error_vars[NORTH_BOUNDARY:]], 0)

    # Culvert is a pain in the ass
    add_dirt_constraint(model, runway_vars[365:373], runway_coords[365:373], 50)

    # Define absolute error
    abs_error_terms_goal = []
    for v,e,r in zip(all_runway_vars, all_abs_error_vars, all_runway_coords):
        error_term = model.NewIntVar(-MAX_HEIGHT, MAX_HEIGHT, "error_" + v.Name())
        model.Add(error_term == v - r[2])
        model.AddAbsEquality(e, error_term)
        if not e.Name().startswith("abs_error_center"):
            abs_error_terms_goal.append(e)


    #change = change_goal(model, abs_error_terms_goal)
    der = derivative_goal(model, east_der_vars, center_der_vars, west_der_vars)
    model.Minimize(der)
    solver = cp_model.CpSolver()

    solver.parameters.log_search_progress = True
    solver.parameters.num_search_workers = 6
    solver.parameters.relative_gap_limit = 0.02
    solver.parameters.max_time_in_seconds = 60

    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        value_in_m = float(solver.ObjectiveValue()) / 1000.0
        num_points = len(center) * 3

        df = get_dataframe(solver, runway_vars, runway_coords)
        plot_dataframe(df)
    else:
        print("Runway fucked.")

if __name__ == "__main__":
    main()