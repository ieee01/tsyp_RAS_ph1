from living_map_gateway.dedup import DedupCache
def test_dedup():
 d=DedupCache(2);assert d.accept((1,2,3));assert not d.accept((1,2,3));assert d.accept((1,2,4));assert d.accept((1,2,5));assert d.accept((1,2,3))
