"""MCP stdio server for Dubins Path Generator."""
import sys
import json
import math

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from client import DubinsPathGenerator

def handle_rpc(request):
    req_id = request.get("id")
    method = request.get("method")
    params = request.get("params", {})

    if method == "tools/list":
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "result": {
                "tools": [
                    {
                        "name": "compute_dubins_path",
                        "description": "Compute optimal Dubins trajectory between two poses (x, y, theta)",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "start": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
                                "goal": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
                                "r_min": {"type": "number", "description": "Minimum turning radius"}
                            },
                            "required": ["start", "goal"]
                        }
                    }
                ]
            }
        }
    elif method == "tools/call":
        name = params.get("name")
        args = params.get("arguments", {})
        if name == "compute_dubins_path":
            start = tuple(args.get("start"))
            goal = tuple(args.get("goal"))
            r_min = float(args.get("r_min", 1.0))
            path = DubinsPathGenerator.shortest_path(start, goal, r_min=r_min)
            return {"jsonrpc": "2.0", "id": req_id, "result": path}
        return {"jsonrpc": "2.0", "id": req_id, "error": {"code": -32601, "message": f"Method {name} not found"}}
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": -32600, "message": "Invalid request"}}

def main():
    for line in sys.stdin:
        if not line.strip():
            continue
        try:
            req = json.loads(line)
            res = handle_rpc(req)
            sys.stdout.write(json.dumps(res) + "\n")
            sys.stdout.flush()
        except Exception as e:
            sys.stdout.write(json.dumps({"jsonrpc": "2.0", "error": {"code": -32700, "message": str(e)}}) + "\n")
            sys.stdout.flush()

if __name__ == "__main__":
    main()
