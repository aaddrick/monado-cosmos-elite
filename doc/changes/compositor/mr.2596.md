---
- mr.2615
- mr.2616
- mr.2617
- mr.2620
- issue.508
---

Support compensation for rolling scanout HMDs. Implements per-scanline timewarp to correct projection distortion on devices with rolling (non-global) refresh patterns. This addresses visual artifacts on HMDs that refresh line-by-line rather than all at once. **Limitations**: Currently only supports top-to-bottom scanout direction.
