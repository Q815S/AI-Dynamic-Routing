import streamlit as st
from streamlit_folium import st_folium
import folium
from folium.plugins import AntPath
import requests
import polyline
import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# 1. API 및 유틸리티 (캐싱 및 안정성 강화)

@st.cache_data(ttl=86400, show_spinner=False)
def fetch_stops_goyang():
    url = "http://overpass-api.de/api/interpreter"
    query = """
    [out:json][timeout:60];
    area["name"="고양시"]["admin_level"~"4|6|8"]->.a;
    (node["highway"="bus_stop"](area.a););
    out body;
    """
    try:
        r = requests.get(url, params={'data': query}, timeout=60)
        data = r.json()
        return [{"name": e.get('tags', {}).get('name', '정류장'), "coords": (e['lat'], e['lon'])} 
                for e in data.get('elements', [])]
    except:
        return []

def get_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi, dlambda = math.radians(lat2-lat1), math.radians(lon2-lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return int(2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a)))

def get_osrm_data(coords_list):
    """도로망 경로 및 총 거리(m), 시간(s) 반환"""
    if len(coords_list) < 2: return [], 0, 0
    loc_str = ";".join([f"{lon},{lat}" for lat, lon in coords_list])
    url = f"http://router.project-osrm.org/route/v1/driving/{loc_str}?overview=full&geometries=polyline"
    try:
        r = requests.get(url, timeout=10).json()
        route = r["routes"][0]
        return polyline.decode(route["geometry"]), route["distance"], route["duration"]
    except:
        return coords_list, 0, 0

# 2. 실시간 엔진 및 최적화

def update_system():
    if not st.session_state['passengers']:
        st.session_state.update({'current_route_path': None, 'summary': [], 'metrics': (0, 0)})
        return
    
    sol, path_coords = solve_vrp(st.session_state['depot'], st.session_state['passengers'])
    if sol:
        road, dist, dur = get_osrm_data(path_coords)
        st.session_state['current_route_path'] = road
        st.session_state['metrics'] = (dist / 1000, dur / 60) # km, min 변환
        
        names = []
        for c in path_coords:
            if c == st.session_state['depot']: names.append("차고지")
            else:
                n = next((s['name'] for s in st.session_state['nearby_stops'] if s['coords'] == c), "정류장")
                names.append(n)
        st.session_state['summary'] = names

def solve_vrp(depot, passengers):
    locs = [depot]
    pairs = []
    for p in passengers:
        p_idx = len(locs); locs.append(p['start_stop']['coords'])
        d_idx = len(locs); locs.append(p['end_stop']['coords'])
        pairs.append((p_idx, d_idx))

    mgr = pywrapcp.RoutingIndexManager(len(locs), 1, 0)
    model = pywrapcp.RoutingModel(mgr)
    def dist_fn(f, t): return get_distance(*locs[mgr.IndexToNode(f)], *locs[mgr.IndexToNode(t)])
    t_idx = model.RegisterTransitCallback(dist_fn)
    model.SetArcCostEvaluatorOfAllVehicles(t_idx)
    model.AddDimension(t_idx, 0, 100000, True, "Dist")
    d_dim = model.GetDimensionOrDie("Dist")
    for p, d in pairs:
        pi, di = mgr.NodeToIndex(p), mgr.NodeToIndex(d)
        model.AddPickupAndDelivery(pi, di)
        model.solver().Add(model.VehicleVar(pi) == model.VehicleVar(di))
        model.solver().Add(d_dim.CumulVar(pi) <= d_dim.CumulVar(di))

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    solution = model.SolveWithParameters(params)
    if solution:
        path = []; i = model.Start(0)
        while not model.IsEnd(i):
            path.append(locs[mgr.IndexToNode(i)]); i = solution.Value(model.NextVar(i))
        path.append(locs[mgr.IndexToNode(i)])
        return solution, path
    return None, []

# 3. UI 메인 레이아웃

def main():
    st.set_page_config(layout="wide", page_title="고양특례시 AI Dynamic Routing")
    st.title("고양특례시 AI Dynamic Routing")

    if 'init' not in st.session_state:
        with st.spinner("정류장 데이터 로드 중..."):
            stops = fetch_stops_goyang()
        st.session_state.update({
            'init': True, 'depot': (37.661545, 126.747219), 'passengers': [],
            'nearby_stops': stops, 'temp_p': None, 'current_route_path': None,
            'summary': [], 'metrics': (0, 0), 'last_click': None, 'mode': 'idle'
        })

    col_map, col_ctrl = st.columns([0.7, 0.3])

    # --- 우측 제어 패널 ---
    with col_ctrl:
        st.subheader("제어 센터")
        c1, c2 = st.columns(2)
        if c1.button("차고지 편집", use_container_width=True): 
            st.session_state['mode'] = 'depot'
            st.toast("차고지 편집 모드 활성화")
        if c2.button("탑승객 추가", use_container_width=True): 
            st.session_state['mode'] = 'pass'
            st.toast("승객 추가 모드 활성화")

        if st.button("전체 데이터 초기화", use_container_width=True, type="primary"):
            st.session_state.update({'passengers': [], 'temp_p': None, 'current_route_path': None, 'summary': [], 'metrics': (0,0), 'mode': 'idle'})
            st.rerun()

        st.divider()
        
        st.subheader("탑승객 리스트")
        if not st.session_state['passengers']:
            st.caption("대기 승객이 없습니다.")
        else:
            for i, p in enumerate(st.session_state['passengers']):
                with st.container(border=True):
                    st.markdown(f"**승객 {i+1}**")
                    st.caption(f"{p['start_stop']['name']} → {p['end_stop']['name']}")
                    if st.button(f"삭제", key=f"del_{i}", use_container_width=True):
                        st.session_state['passengers'].pop(i); update_system(); st.rerun()

    # --- 좌측 지도 패널 ---
    with col_map:
        m = folium.Map(location=st.session_state['depot'], zoom_start=14)
        
        # 차고지 근처 정류장만 지도에 표시 (성능 최적화)
        visible_stops = [s for s in st.session_state['nearby_stops'] 
                         if get_distance(*st.session_state['depot'], *s['coords']) < 3000]
        
        for s in visible_stops:
            folium.CircleMarker(s['coords'], radius=2, color='blue', fill=True, opacity=0.4).add_to(m)
        
        folium.Marker(st.session_state['depot'], icon=folium.Icon(color='black', icon='home'), tooltip="차고지").add_to(m)
        for i, p in enumerate(st.session_state['passengers']):
            folium.Marker(p['start_stop']['coords'], icon=folium.Icon(color='green', icon='user'), tooltip=f"P{i+1} 탑승").add_to(m)
            folium.Marker(p['end_stop']['coords'], icon=folium.Icon(color='red', icon='flag'), tooltip=f"P{i+1} 하차").add_to(m)
        
        if st.session_state['temp_p']:
            folium.Marker(st.session_state['temp_p']['coords'], icon=folium.Icon(color='orange'), tooltip="탑승지 선택됨").add_to(m)

        if st.session_state['current_route_path']:
            AntPath(locations=st.session_state['current_route_path'], color="blue", pulse_color="#FFFFFF", delay=800, weight=6).add_to(m)

        out = st_folium(m, width="100%", height=550, key="master_map")

        if out and out.get('last_clicked'):
            pos = (out['last_clicked']['lat'], out['last_clicked']['lng'])
            if pos != st.session_state['last_click']:
                st.session_state['last_click'] = pos
                if not st.session_state['nearby_stops']: st.error("데이터 로드 중...")
                else:
                    if st.session_state['mode'] == 'depot':
                        st.session_state['depot'] = pos; update_system(); st.rerun()
                    elif st.session_state['mode'] == 'pass':
                        target = min(st.session_state['nearby_stops'], key=lambda x: get_distance(*pos, *x['coords']))
                        if st.session_state['temp_p'] is None: st.session_state['temp_p'] = target
                        else:
                            st.session_state['passengers'].append({"start_stop": st.session_state['temp_p'], "end_stop": target})
                            st.session_p = None; st.session_state['temp_p'] = None; update_system()
                        st.rerun()

        # --- 하단 분석 데이터 ---
        if st.session_state['summary']:
            st.divider()
            d, t = st.session_state['metrics']
            c1, c2 = st.columns(2)
            c1.metric("총 운행 거리", f"{d:.2f} km")
            c2.metric("예상 소요 시간", f"{int(t)} 분")
            st.info(" ➔ ".join([f"**{n}**" for n in st.session_state['summary']]))

if __name__ == '__main__':
    main()