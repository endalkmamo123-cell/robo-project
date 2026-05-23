#!/usr/bin/env python3
import json
from collections import defaultdict

def main():
    results = defaultdict(list)
    try:
        with open('/tmp/algo_results.jsonl') as f:
            for line in f:
                line = line.strip()
                if line:
                    r = json.loads(line)
                    results[r['algorithm']].append(r)
    except FileNotFoundError:
        print("No results yet. Run A* and Dijkstra first by sending /goal_pose.")
        return

    sep = '=' * 65
    print()
    print(sep)
    print("  ALGORITHM COMPARISON RESULTS")
    print(sep)
    print(f"{'Algorithm':<12} {'Runs':<6} {'Avg Time(s)':<14} {'Avg Path(m)':<14} {'Avg Nodes'}")
    print('-' * 65)

    for algo, runs in results.items():
        avg_t = sum(r['time_s'] for r in runs) / len(runs)
        avg_d = sum(r['path_distance_m'] for r in runs) / len(runs)
        avg_n = sum(r['nodes_visited'] for r in runs) / len(runs)
        print(f"{algo:<12} {len(runs):<6} {avg_t:<14.4f} {avg_d:<14.3f} {avg_n:<12.0f}")

    print(sep)
    print()

    if 'A*' in results and 'Dijkstra' in results:
        a_nodes = sum(r['nodes_visited'] for r in results['A*']) / len(results['A*'])
        d_nodes = sum(r['nodes_visited'] for r in results['Dijkstra']) / len(results['Dijkstra'])
        savings = (1 - a_nodes / d_nodes) * 100 if d_nodes > 0 else 0
        print(f"KEY INSIGHT: A* explored {savings:.1f}% fewer nodes than Dijkstra")
        print()

if __name__ == '__main__':
    main()
