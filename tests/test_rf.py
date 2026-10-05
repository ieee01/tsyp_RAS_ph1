from living_map_radio.model import RFModel, delivery_probability


def test_piecewise_rf_curve():
    assert delivery_probability(0) == 0.98
    assert delivery_probability(12) == 0.98
    assert 0.59 < delivery_probability(18) < 0.61
    assert 0.099 < delivery_probability(22) < 0.101
    assert delivery_probability(22.01) == 0.0
    assert delivery_probability(30) == 0.0


def test_seed_makes_delivery_reproducible():
    a = RFModel(seed=42)
    b = RFModel(seed=42)
    assert [a.deliver(19)[0] for _ in range(8)] == [b.deliver(19)[0] for _ in range(8)]
