"""A second publisher on a watched single-writer topic: that component stops beating -> stop. Removing it recovers."""

import signal
import time

import ohk_harness as h
import ohk_nav_harness as n

CACHE_S = 0.5  # publisher count is refreshed from callbacks at most every 0.5 s
DISCOVERY_S = 0.2
# second publisher appears -> discovery -> cache refresh -> lidar max_age -> aggregator tick -> soft stop -> margin
BOUND_S = DISCOVERY_S + CACHE_S + n.SCAN_MAX_AGE + 0.05 + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S


def generate_test_description():
    return n.build_nav_description()


class TestSecondPublisher(n.NavCase):
    def test_second_writer_stops_the_watch(self):
        self.wait_idle_run()
        t_spawn = time.monotonic()
        extra = self.spawn_fake("scan")
        # t0 = the moment the second writer is visible in the graph (python start-up is not the system's latency).
        # The watcher sees the writer as early as the test does, so the stop can precede t0: latency is then negative.
        self.wait_for(lambda: self.probe.count_publishers("/scan") >= 2, 10.0, "second /scan publisher visible")
        t0 = time.monotonic()
        self.wait_for(lambda: self.first_zero_since(t_spawn) is not None, BOUND_S + 1.0, "stop after second publisher")
        lat = self.first_zero_since(t_spawn) - (t0 - t_spawn)
        self.record("r3_second_publisher", from_visible_ms=lat * 1000, from_spawn_ms=(lat + t0 - t_spawn) * 1000,
                    bound_from_visible_ms=BOUND_S * 1000)
        self.assertLess(lat, BOUND_S)
        self.assert_blames("lidar", t_spawn)
        with self.probe.lock:
            late = [c for ts, c, _ in self.probe.comp_details if c == "lidar" and ts > t0 + CACHE_S + DISCOVERY_S + 0.3]
        self.assertEqual(late, [], "lidar kept beating with two writers")
        t1 = time.monotonic()
        extra.send_signal(signal.SIGINT)  # graceful: a SIGKILLed participant lingers in discovery for its lease
        extra.wait(5)
        self.wait_run(10.0)
        self.record("r3_second_publisher_recovery", seconds=time.monotonic() - t1)
