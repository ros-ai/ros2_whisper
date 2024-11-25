# Transcript Manager Package

This package fuses overlapping output from the whisper.cpp engine to allow boarder-less streaming.  

The intended usage is running whisper repeatedly over a (e.g.) 10 second audio ring, once every second.  New audio data (the next second) is then added to the transcript while previous audio data (last 9 seconds) are used  for text alignment and redundancy.  



## Operation

### Deserialization

Input to the node comes from the WhisperToken.msg.

As an example, the message contains ('|' token boundary):

```
Segment Tokens: 20  Duration: 568  Data: 
  [_BEG_]| In| consequence|,| I|'m| inclined| to| reserve| all| judgments|,| a| habit| that| has| opened| up| many|[_TT_284]
Segment Tokens: 17  Duration: 432  Data:  
   curious| n|atures| to| me| and| also| made| me| the| victim| of| not| a| few|.|[_TT_500]

```

Individual tokens are fused together by whitespace or punctuation to form words.  The result would be ('||' word boundary):

```
[2024-11-18 12:17:57.685(5680 ms)]:   In consequence, I'm inclined to reserve all judgments, a habit that has opened up many
[2024-11-18 12:18:03.365(4320 ms)]:   curious natures to me and also made me the victim of not a few.

```

- Where the tokens, segment boundaries and duration estimates are output from whisper.cpp



### Longest Common Substring with Gaps (LCS)

The first step in merging is finding an alignment with incoming words and the existing transcript.  

This is done by computing the longest common substring between the two, with a predefined number of (gaps) between matching words.  The gaps exist so that similar sounding but different words (e.g. "6" v.s. "six") can still exist in the longest common substring, even if they do not match.   

Gaps in the substring can also be caused by

- An word being in part of the update and not in the transcript (which will be inserted)
- A word existing in the transcript that is not part of the update (its occurrence count would be decremented).

As an example:

```
[Existing Transcript]:   Once up on a time
[New Update Data]:       Once upon a time
```

Matched pairs from the LCS algorithm will be:

```
 (0, 0) // Once
 (3, 2) // a
 (4, 3) // time
```



The exact matching for individual words is made easier by:

1. Converting all characters to lower-case.
2. Removing (often leading) whitespace from words.
3. Not attempting to match punctuation.



### Merging Strategy

After finding pairs of indices which contain matches in the longest common substring, information from the update can be merged into the transcript.

Starting at the first match in the LCS and going until either the next match or the last word of the transcript AND update, we increment each word index to handle the following cases:

| -----                 | Example                 | Operation                                           | Description                                                  |
| --------------------- | ----------------------- | --------------------------------------------------- | ------------------------------------------------------------ |
| Exact Word Match      | "Hello" == "Hello"      | Increment Occurrence Counter. (2x)                  |                                                              |
| Loose Word Match      | " world" ~= "World"     | Add Conflict Entry<br />Increase Occurrence Counter | Each word tracks conflicts (different words/capitalization/...). <br />Adding a conflict entry starts at 1 or increments it if it exists. |
| Different Words       | "teacher" != "preacher" | Add Conflict Entry                                  | If a conflict gains more occurrences than the original word, it is replaced. |
| Punctuation v.s. Word | "."  v.s. "discussing"  | Decrement Punctuation Entry<br />Add Conflict Entry | A common occurrence is the transcript ending in "..." as the audio gets cut off.  First decrement the punctuation occurrence counter ---which if moved to 0, will cause it to be replaced with the conflict. |
| Missing Word          | ""  v.s. "egg-salad"    | Insert New Word                                     | An word is inserted in the transcript with one occurrence.   |
| Extra Word            | "on" v.s. ""            | Decrement Occurrence Counter                        | If all occurrence counters (inc. conflicts) reach less than "-1", the word and conflicts are permanently removed from the transcript. |

- A similar strategy is used to merge the segment boundaries -- incrementing if the boundaries match and decrementing/removing or inserting mismatches between the transcript and update.
  - Timing information contained in the segment is replaced by the update.  

- After matching/incrementing/decrementing, we parse through the transcript and remove words with equal or fewer than "-1" occurrences.



### Stale Segment Marker

Since the update contains only what was recent said, it helps to freeze earlier parts of the transcript and only use the latest parts for computing the LCS.  

Before getting a match-able string from the transcript to compare with the update, we set all segments to stale that have a timestamp earlier than the first timestamp in the update.  In practice it helps to include one segment before this, since the update may still contain half of the first segment.

