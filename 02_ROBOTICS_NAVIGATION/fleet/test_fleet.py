import json
import unittest
from telemetry import connect_store, decode, metrics, store_sample


class TelemetryTests(unittest.TestCase):
    def test_duplicate_and_out_of_order_messages_preserve_sequence_gaps(self):
        database = connect_store(":memory:")
        for sequence in [3, 0, 3, 1]:
            sample = decode(
                json.dumps(
                    {
                        "robot": "alpha",
                        "sequence": sequence,
                        "sent_ns": 1000,
                        "x": 0,
                        "y": 0,
                        "battery": 80,
                    }
                )
            )
            store_sample(database, sample, 2000)
        summary = metrics(database, {"alpha": 5})["alpha"]
        self.assertEqual(summary["missing_sequences"], [2, 4])
        self.assertEqual(summary["received"], 3)
        self.assertEqual(summary["mean_delay_ms"], 0.001)
        database.close()

    def test_malformed_numeric_and_timestamp_payloads_are_rejected(self):
        sample = {
            "robot": "alpha",
            "sequence": 1,
            "sent_ns": 1000,
            "x": 0,
            "y": 0,
            "battery": 80,
        }
        for field, invalid in [
            ("sequence", 2**63),
            ("sent_ns", 2**63),
            ("battery", 101),
            ("x", float("nan")),
            ("sequence", True),
            ("robot", ""),
        ]:
            with self.subTest(field=field), self.assertRaises(ValueError):
                decode(json.dumps(dict(sample, **{field: invalid})))
        database = connect_store(":memory:")
        with self.assertRaises(ValueError):
            store_sample(database, sample, 999)
        database.close()


if __name__ == "__main__":
    unittest.main()
