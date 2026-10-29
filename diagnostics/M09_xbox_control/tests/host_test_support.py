def assert_stream_equivalent(case, actual, expected):
    """Independent scheduler may dispatch one UI tick earlier/later.

    Compare command ordering and contents, with a 25 ms scheduling tolerance.
    Celestial angles move with calculation time; permit only the corresponding
    sub-millidegree difference, never a different request/sequence/target.
    """
    case.assertEqual(len(actual), len(expected))
    for (at, data), (want_at, want_data) in zip(actual, expected):
        case.assertAlmostEqual(at, want_at, delta=.025)
        if data.startswith((b"CELESTIAL_GOTO ", b"CELESTIAL_UPDATE ")):
            fields, want = data.split(), want_data.split()
            case.assertEqual(fields[:-2], want[:-2])
            for value, target in zip(fields[-2:], want[-2:]):
                case.assertAlmostEqual(float(value), float(target), delta=.0002)
        else:
            case.assertEqual(data, want_data)
