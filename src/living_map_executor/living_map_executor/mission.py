def select_target(memories,target_event_type=1):
    valid=[m for m in memories if m.get('event_type')==target_event_type and m.get('freshness')!='EXPIRED']
    return max(valid,key=lambda m:(m.get('confidence',0),-m.get('age_s',0)),default=None)
