#!/usr/bin/env python3
# Task 4 owner: Tan Chew Miang Edwin (A0201867A), Group 13.
"""Offline tests for the Task 4 speech->Task 3 decision logic (no ROS, no mic, no network).

classify_transcript() is the pure function speech_command.py calls on whatever
text the STT engine returned; these tests stand in for a live microphone by
feeding it text a recogniser would plausibly produce. They do not exercise
recognize_google/recognize_sphinx or rclpy itself.
"""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from speech_command import classify_transcript  # noqa: E402


def test_single_colour_with_find_prefix():
    outcome, payload = classify_transcript('find red')
    assert outcome == 'ok'
    assert payload == ['Red']


def test_two_colours_and_join():
    outcome, payload = classify_transcript('find red and blue')
    assert outcome == 'ok'
    assert payload == ['Red', 'Blue']


def test_bare_list_allowed_for_speech_without_find_prefix():
    # Typed commands must start with "find"; spoken commands may omit it
    # because allow_bare_list=True, matching task3_mission.py's on_command(speech=True).
    outcome, payload = classify_transcript('red and blue')
    assert outcome == 'ok'
    assert payload == ['Red', 'Blue']


def test_four_colours_oxford_comma_and_block_word():
    outcome, payload = classify_transcript(
        'please find the red block, the blue block, yellow and green')
    assert outcome == 'ok'
    assert payload == ['Red', 'Blue', 'Yellow', 'Green']


def test_trailing_punctuation_from_recognizer_is_tolerated():
    outcome, payload = classify_transcript('find red and blue.')
    assert outcome == 'ok'
    assert payload == ['Red', 'Blue']


def test_empty_transcript_is_rejected():
    outcome, reason = classify_transcript('')
    assert outcome == 'error'
    assert reason


def test_non_english_transcript_is_rejected():
    outcome, reason = classify_transcript('encuentra el rojo')
    assert outcome == 'error'
    assert reason


def test_more_than_four_colours_is_rejected():
    outcome, reason = classify_transcript('find red, blue, yellow, green and purple')
    assert outcome == 'error'
    assert reason


def test_repeated_colour_is_rejected():
    outcome, reason = classify_transcript('find red and red')
    assert outcome == 'error'
    assert reason


def test_unknown_colour_word_is_rejected():
    outcome, reason = classify_transcript('find pink')
    assert outcome == 'error'
    assert reason


def test_cancel_is_not_this_nodes_concern():
    # "cancel"/"stop" are handled by task3_mission.on_command itself (any topic);
    # this node only classifies to decide whether to forward and log [STT] colours.
    outcome, reason = classify_transcript('cancel')
    assert outcome == 'error'
    assert reason


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
