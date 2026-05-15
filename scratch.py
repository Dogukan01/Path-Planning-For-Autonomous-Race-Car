import requests

def fetch_track(name):
    query = f"""
    [out:json][timeout:25];
    way["name"~"{name}", i]["highway"="raceway"];
    out geom;
    """
    url = "https://overpass-api.de/api/interpreter"
    headers = {
        'User-Agent': 'AutonomousRaceCarSim/1.0 (test@example.com)'
    }
    try:
        response = requests.post(url, data={'data': query}, headers=headers, timeout=10)
        print("Status code:", response.status_code)
        if response.status_code != 200:
            print("Response:", response.text[:200])
        data = response.json()
        ways = data.get('elements', [])
        if not ways:
            print("No tracks found.")
            return
        
        for way in ways:
            geom = way.get('geometry', [])
            print(f"Track: {way.get('tags', {}).get('name')}, Nodes: {len(geom)}")
    except Exception as e:
        print("Error:", e)

fetch_track("Monza")
