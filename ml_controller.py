import joblib
import pandas as pd
import subprocess
import random

# Load pre-trained ML artifacts
scaler = joblib.load('manet_scaler.pkl')
knn_models = joblib.load('knn_optimal_models.pkl')
feature_cols = [
    'Node_Count', 'Node_Density_nodes_per_km2', 'Terrain_Size_m',
    'Node_Speed_mps', 'Traffic_Rate_bps', 'Packet_Size_bytes', 'TTL',
    'Wait_Time_s', 'Retry_Attempts', 'Route_Expiry_s', 'Buffer_Size_packets',
    'Thresh_Min', 'Thresh_Max', 'Drop_Probability'
]

def generate_random_environment():
    n_nodes = random.randint(30, 100)
    terrain = random.choice([1000, 1200, 1500])
    density = round(n_nodes / ((terrain / 1000.0) ** 2), 2)
    return {
        'Node_Count': n_nodes,
        'Node_Density_nodes_per_km2': density,
        'Terrain_Size_m': terrain,
        'Node_Speed_mps': round(random.uniform(5.0, 30.0), 2),
        'Traffic_Rate_bps': random.choice([1024, 2048, 4096]),
        'Packet_Size_bytes': random.choice([512, 1024]),
        'TTL': random.randint(1, 5),
        'Wait_Time_s': round(random.uniform(0.5, 2.0), 2),
        'Retry_Attempts': random.randint(1, 4),
        'Route_Expiry_s': round(random.uniform(2.0, 5.0), 2),
        'Buffer_Size_packets': random.choice([50, 100, 150]),
        'Thresh_Min': random.randint(5, 10),
        'Thresh_Max': random.randint(15, 30),
        'Drop_Probability': round(random.uniform(0.01, 0.1), 3)
    }

def predict_protocols(env):
    raw_df = pd.DataFrame([env], columns=feature_cols)
    scaled_vector = scaler.transform(raw_df)
    predictions = {}
    predictions['Throughput'] = knn_models['Throughput'].predict(scaled_vector)[0]
    predictions['PDR'] = knn_models['PDR'].predict(scaled_vector)[0]
    predictions['Delay'] = knn_models['Delay'].predict(scaled_vector)[0]
    return predictions

def run_ns3_simulation(env, protocol):
    ns3_cmd = (
        f"./ns3 run 'dset "
        f"--nNodes={env['Node_Count']} "
        f"--terrainSize={env['Terrain_Size_m']} "
        f"--nodeSpeed={env['Node_Speed_mps']} "
        f"--trafficRate={env['Traffic_Rate_bps']}bps "
        f"--packetSize={env['Packet_Size_bytes']} "
        f"--ttl={env['TTL']} "
        f"--waitTime={env['Wait_Time_s']} "
        f"--retryAttempts={env['Retry_Attempts']} "
        f"--routeExpiry={env['Route_Expiry_s']} "
        f"--bufferSize={env['Buffer_Size_packets']} "
        f"--threshMin={env['Thresh_Min']} "
        f"--threshMax={env['Thresh_Max']} "
        f"--dropProb={env['Drop_Probability']} "
        f"--routingProtocol={protocol}'"
    )
    result = subprocess.run(ns3_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.stderr:
        print(f"\n[NS-3 Error]:\n{result.stderr}")
    metrics = {'Throughput': 'N/A', 'PDR': 'N/A', 'Delay': 'N/A'}
    for line in result.stdout.splitlines():
        if "Throughput:" in line:
            metrics['Throughput'] = line.split("Throughput:")[1].strip()
        elif "PDR:" in line:
            metrics['PDR'] = line.split("PDR:")[1].strip()
        elif "E2E Delay:" in line:
            metrics['Delay'] = line.split("E2E Delay:")[1].strip()
    return metrics

def run_ml_controller():
    # Generate one network environment
    env = generate_random_environment()
    print("\n" + "=" * 70)
    print("NETWORK ENVIRONMENT")
    print("=" * 70)
    for key, value in env.items():
        print(f"{key}: {value}")

    # Predict protocols for all three metrics
    predictions = predict_protocols(env)
    print("\n" + "=" * 70)
    print("ML PROTOCOL PREDICTIONS")
    print("=" * 70)
    print(f"Best Protocol for Throughput : {predictions['Throughput']}")
    print(f"Best Protocol for PDR        : {predictions['PDR']}")
    print(f"Best Protocol for Delay      : {predictions['Delay']}")

    # Avoid running the same protocol more than once
    protocols_to_run = list(set(predictions.values()))
    print("\n" + "=" * 70)
    print("NS-3 SIMULATIONS")
    print("=" * 70)
    results = {}
    for protocol in protocols_to_run:
        print(f"\nRunning NS-3 with protocol: {protocol}")
        metrics = run_ns3_simulation(env, protocol)
        results[protocol] = metrics
        print(f"  Throughput : {metrics['Throughput']}")
        print(f"  PDR        : {metrics['PDR']}")
        print(f"  E2E Delay  : {metrics['Delay']}")

    # Final comparison
    print("\n" + "=" * 70)
    print("FINAL RESULTS")
    print("=" * 70)
    print(f"{'Protocol':<15}{'Throughput':<20}{'PDR':<15}{'Delay':<15}")
    print("-" * 65)
    for protocol, metrics in results.items():
        print(f"{protocol:<15}{metrics['Throughput']:<20}{metrics['PDR']:<15}{metrics['Delay']:<15}")
    print("\n" + "=" * 70)
    print("ML RECOMMENDATIONS")
    print("=" * 70)
    print(f"Throughput optimization -> {predictions['Throughput']}")
    print(f"PDR optimization        -> {predictions['PDR']}")
    print(f"Delay optimization       -> {predictions['Delay']}")

if __name__ == "__main__":
    run_ml_controller()
