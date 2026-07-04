import pytest

from upload_handler import sanitize_filename


@pytest.mark.parametrize(
    "name",
    ["mission.db3", "run-2026_07_02.mcap", "bag.zip", "A.db3"],
)
def test_sanitize_accepts_valid_names(name):
    assert sanitize_filename(name) == name


@pytest.mark.parametrize(
    "name",
    [
        "../../etc/passwd.db3",
        "/etc/passwd.db3",
        "..\\windows\\evil.db3",
        "sub/dir/bag.mcap",
        "bag/../other.db3",
    ],
)
def test_sanitize_rejects_path_traversal(name):
    with pytest.raises(ValueError):
        sanitize_filename(name)


@pytest.mark.parametrize("name", ["", "   ", ".", ".."])
def test_sanitize_rejects_empty_and_dots(name):
    with pytest.raises(ValueError):
        sanitize_filename(name)


@pytest.mark.parametrize("name", ["bag.txt", "bag.exe", "bag", "bag.db3.exe"])
def test_sanitize_rejects_bad_extensions(name):
    with pytest.raises(ValueError):
        sanitize_filename(name)


@pytest.mark.parametrize("name", ["bag$.db3", "b a g.mcap", "bag;rm.db3"])
def test_sanitize_rejects_special_chars(name):
    with pytest.raises(ValueError):
        sanitize_filename(name)
