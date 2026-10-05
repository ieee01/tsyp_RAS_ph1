from living_map_executor.mission import select_target
def test_select_target_prefers_confidence_and_excludes_expired():
 m=[{'id':'a','event_type':1,'freshness':'FRESH','confidence':.8,'age_s':10},{'id':'b','event_type':1,'freshness':'FRESH','confidence':.95,'age_s':50},{'id':'c','event_type':1,'freshness':'EXPIRED','confidence':1.0,'age_s':999}];assert select_target(m)['id']=='b'
