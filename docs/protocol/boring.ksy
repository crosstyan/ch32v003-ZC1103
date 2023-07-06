meta:
  id: boring
  title: Boring
  endian: be
  imports: 
    - message_wrapper
doc: |
  `boring` does nothing but to set LED to certain value with a comment.
seq:
  # - id: header
  #   contents: message_wrapper.header
  - id: magic
    contents: [0x50]
  - id: led
    type: u1
    doc: |
      only the lower 3 bits are used. (rgb binary)
  - id: num_comments
    type: u1
  - id: comments
    type: str
    encoding: utf-8
    size: num_comments
