using UnityEngine;

/// <summary>
/// 3-DOF Spherical Manipulator (R-R-P) — DH Convention
/// The arm sweeps upper hemisphere only (θ2: -90→+90 = horizontal→up→horizontal).
/// The column/housing can be substantial since the arm never points downward.
/// Desktop keyboard + Mobile touch. Build target: Android / iOS / Desktop.
/// </summary>
public class SphericalRobotController : MonoBehaviour
{
    [Header("Joint Variables")]
    [Range(-180f, 180f)] public float theta1Deg = 0f;
    [Range(-90f, 90f)]   public float theta2Deg = 55f;
    [Range(0.20f, 0.90f)] public float d3 = 0.55f;

    [Header("Control")]
    public float thetaSpeed = 55f;
    public float d3Speed = 0.35f;
    public int maxTrailPoints = 1200;

    [Header("Materials")]
    public Material baseMaterial;   // assign a URP/Lit material in Inspector

    float d1, theta1Min, theta1Max, theta2Min, theta2Max, d3Min, d3Max;
    const float BEAM_S = 0.10f;

    Transform ttPiv, shPiv, slPiv, eeTip;
    Transform beam, rT, rB, rL, rR, bCap, bInner;
    LineRenderer trail; int trailIdx;
    Vector3 eePosDH;

    enum Mode { Menu, Manual, AutoShell, Auto3DOF, AutoSingle }
    Mode mode = Mode.Menu; bool paused; int aIdx;
    float[] aTh1, aTh2, aD3; float w1, w2, v3f;

    // touch
    int joyFinger = -1; Vector2 joyOrigin, joyDelta; float joyRadius;
    bool touchExt, touchRet;
    int camFinger = -1; Vector2 camPrev;
    float camYaw = -35f, camPitch = 22f, camDist = 1.4f;
    Vector3 camTarget = new Vector3(0, 0.22f, 0);

    // palette
    static Color C(int r, int g, int b, int a = 255) => new Color(r / 255f, g / 255f, b / 255f, a / 255f);
    static readonly Color CARBON      = C(24, 24, 28);
    static readonly Color DARK_STEEL  = C(38, 40, 46);
    static readonly Color MID_STEEL   = C(62, 65, 72);
    static readonly Color STEEL       = C(95, 98, 108);
    static readonly Color LIGHT_STEEL = C(135, 138, 148);
    static readonly Color SILVER      = C(175, 178, 188);
    static readonly Color CHROME      = C(210, 213, 220);
    static readonly Color CYAN_A      = C(0, 195, 235);
    static readonly Color CYAN_G      = C(0, 225, 255);
    static readonly Color CYAN_DIM    = C(0, 120, 160);
    static readonly Color ORANGE      = C(255, 115, 0);
    static readonly Color ORANGE_D    = C(200, 80, 0);
    static readonly Color YELLOW      = C(245, 215, 15);
    static readonly Color RED_S       = C(225, 35, 30);
    static readonly Color GREEN_A     = C(50, 195, 115);
    static readonly Color GREEN_D     = C(32, 155, 82);
    static readonly Color PURPLE      = C(140, 65, 185);
    static readonly Color PURPLE_D    = C(105, 45, 140);
    static readonly Color RUBBER      = C(18, 18, 22);
    static readonly Color NAVY        = C(18, 28, 60);
    static readonly Color WHITE_L     = C(235, 237, 242);
    static readonly Color TEAL_TRAIL  = new Color(0, 0.80f, 0.94f, 0.85f);

    float Sc => Mathf.Max(Screen.dpi, 160f) / 160f;
    bool Mobile => Application.platform == RuntimePlatform.Android ||
                   Application.platform == RuntimePlatform.IPhonePlayer || Input.touchSupported;

    // ── Perf cache: camera only ──
    Camera cam;

    // Trail throttle
    int trailFrame;
    int trailFrameSkip = 1;
    float trailMinDist = 0.0025f;
    float trailMinDistSqr;
    Vector3 trailLast;
    bool trailHasLast;

    // GUI style cache (reduces OnGUI allocations)
    float guiScaleCached = -1f;
    GUIStyle styTitle, stySub, styHudT, styHudL, styHudD;
    GUIStyle btnMain, btnTouch, btnSmall, btnPause;

    // ═══ LIFECYCLE ═══
    void Awake()
    {
        d1 = 0.35f; theta1Min = -180; theta1Max = 180;
        theta2Min = -90; theta2Max = 90; d3Min = 0.20f; d3Max = 0.90f;
        joyRadius = Mathf.Max(Screen.dpi, 160) * 0.4f;

        // Cache camera (Camera.main is slower on mobile if called repeatedly).
        cam = Camera.main;
        if (!cam) cam = FindObjectOfType<Camera>();

        // Mobile-safe defaults (keeps Windows OK).
        if (Mobile)
        {
            maxTrailPoints = Mathf.Clamp(maxTrailPoints, 200, 700);
            trailFrameSkip = 2;
            trailMinDist = 0.0040f;
        }
        trailMinDistSqr = trailMinDist * trailMinDist;

        BuildRobot(); SetupTrail(); SetupCam(); BuildEnv();
        // Ensure the initial pose and camera are correct even on the Menu screen.
        Clamp(); ApplyFK(); UpdateCamNow();
    }

    void Update()
    {
        HandleTouch();
        // Keep camera orbit responsive even while on Menu.
        OrbitCam();

        if (mode == Mode.Manual) DoInput();
        else if (mode != Mode.Menu && !paused) DoAuto();

        if (mode != Mode.Menu && (Input.GetKeyDown(KeyCode.Space) || Input.GetKeyDown(KeyCode.P))) paused = !paused;
        if (Input.GetKeyDown(KeyCode.Escape) || Input.GetKeyDown(KeyCode.Backspace)) mode = Mode.Menu;

        Clamp();
        ApplyFK();
        if (mode != Mode.Menu) DoTrail();
    }

    // ═══ FK ═══
    void ApplyFK()
    {
        float t1r = theta1Deg * Mathf.Deg2Rad, t2r = theta2Deg * Mathf.Deg2Rad;
        float c1 = Mathf.Cos(t1r), s1 = Mathf.Sin(t1r), c2 = Mathf.Cos(t2r), s2 = Mathf.Sin(t2r);
        eePosDH = new Vector3(d3 * c1 * s2, d3 * s1 * s2, d3 * c2 + d1);

        if (ttPiv) ttPiv.localRotation = Quaternion.Euler(0, theta1Deg, 0);
        if (shPiv) shPiv.localRotation = Quaternion.Euler(0, 0, 90f - theta2Deg);
        if (slPiv) slPiv.localPosition = V(d3, 0, 0);

        float bE = d3 + 0.06f, bL = bE - BEAM_S, bM = (BEAM_S + bE) * 0.5f;
        Sx(beam, bM, bL, 0, 0); Sx(rT, bM, bL, 0.040f, 0); Sx(rB, bM, bL, -0.040f, 0);
        Sx(rL, bM, bL, 0, 0.040f); Sx(rR, bM, bL, 0, -0.040f);
        if (bCap) bCap.localPosition = V(bE + 0.008f, 0, 0);
        float iL = Mathf.Max(d3 - 0.04f, 0.02f);
        if (bInner) { bInner.localPosition = V(BEAM_S + iL * 0.5f, 0, 0); var s = bInner.localScale; s.x = iL; bInner.localScale = s; }
    }

    void Sx(Transform t, float cx, float sx, float yo, float zo)
    { if (!t) return; t.localPosition = V(cx, yo, zo); var s = t.localScale; s.x = sx; t.localScale = s; }

    // ═══ INPUT ═══
    void DoInput()
    {
        float h = Input.GetAxisRaw("Horizontal") + joyDelta.x;
        float v = Input.GetAxisRaw("Vertical") + joyDelta.y;
        theta1Deg += h * thetaSpeed * Time.deltaTime;
        theta2Deg -= v * thetaSpeed * Time.deltaTime;
        float dd = (Input.GetKey(KeyCode.E) ? 1f : 0f) - (Input.GetKey(KeyCode.Q) ? 1f : 0f);
        if (touchExt) dd += 1f; if (touchRet) dd -= 1f;
        d3 += dd * d3Speed * Time.deltaTime;
    }
    void Clamp()
    { theta1Deg = Mathf.Clamp(theta1Deg, theta1Min, theta1Max); theta2Deg = Mathf.Clamp(theta2Deg, theta2Min, theta2Max); d3 = Mathf.Clamp(d3, d3Min, d3Max); }

    // ═══ TOUCH ═══
    void HandleTouch()
    {
        if (mode != Mode.Manual) { joyFinger = -1; joyDelta = Vector2.zero; return; }
        for (int i = 0; i < Input.touchCount; i++)
        {
            Touch t = Input.GetTouch(i); float hw = Screen.width * 0.5f;
            if (t.phase == TouchPhase.Began && t.position.x < hw && t.position.y < Screen.height * 0.55f && joyFinger == -1)
            { joyFinger = t.fingerId; joyOrigin = t.position; joyDelta = Vector2.zero; }
            if ((t.phase == TouchPhase.Moved || t.phase == TouchPhase.Stationary) && t.fingerId == joyFinger)
            { var d = t.position - joyOrigin; if (d.magnitude > joyRadius) d = d.normalized * joyRadius; joyDelta = d / joyRadius; }
            if ((t.phase == TouchPhase.Ended || t.phase == TouchPhase.Canceled) && t.fingerId == joyFinger)
            { joyFinger = -1; joyDelta = Vector2.zero; }
        }
    }

    void OrbitCam()
    {
        for (int i = 0; i < Input.touchCount; i++)
        {
            Touch t = Input.GetTouch(i); if (t.fingerId == joyFinger) continue;
            if (t.phase == TouchPhase.Began && t.position.x > Screen.width * 0.5f && t.position.y > Screen.height * 0.3f)
            { camFinger = t.fingerId; camPrev = t.position; }
            if (t.fingerId == camFinger)
            {
                if (t.phase == TouchPhase.Moved) { var d = t.position - camPrev; camYaw += d.x * 0.15f; camPitch -= d.y * 0.15f; camPitch = Mathf.Clamp(camPitch, 5, 80); camPrev = t.position; }
                if (t.phase == TouchPhase.Ended || t.phase == TouchPhase.Canceled) camFinger = -1;
            }
        }
        if (Input.touchCount == 2) { Touch a = Input.GetTouch(0), b = Input.GetTouch(1); float pv = ((a.position - a.deltaPosition) - (b.position - b.deltaPosition)).magnitude; float cv = (a.position - b.position).magnitude; camDist -= (cv - pv) * 0.002f; camDist = Mathf.Clamp(camDist, 0.5f, 4); }
        if (Input.GetMouseButton(1)) { camYaw += Input.GetAxis("Mouse X") * 3; camPitch -= Input.GetAxis("Mouse Y") * 3; camPitch = Mathf.Clamp(camPitch, 5, 80); }
        camDist -= Input.GetAxis("Mouse ScrollWheel") * 0.5f; camDist = Mathf.Clamp(camDist, 0.5f, 4);
        UpdateCamNow();
    }

    // ═══ AUTO ═══
    void MkShell() { int n1 = 80, n2 = 40, t = n1 * n2; aTh1 = new float[t]; aTh2 = new float[t]; aD3 = new float[t]; var v1 = Lin(theta1Min, theta1Max, n1); var v2 = Lin(theta2Min, theta2Max, n2); int k = 0; bool f = true; for (int j = 0; j < n2; j++) { for (int i = 0; i < n1; i++) { int ii = f ? i : n1 - 1 - i; aTh1[k] = v1[ii]; aTh2[k] = v2[j]; aD3[k] = d3Max; k++; } f = !f; } aIdx = 0; }
    void Mk3() { w1 = 40; w2 = 52; v3f = 0.12f; theta1Deg = 0; theta2Deg = 0; d3 = (d3Min + d3Max) * 0.5f; }
    void Mk1(int j) { w1 = j == 0 ? 50 : 0; w2 = j == 1 ? 60 : 0; v3f = j == 2 ? 0.2f : 0; theta1Deg = 0; theta2Deg = 0; d3 = (d3Min + d3Max) * 0.5f; }
    void DoAuto()
    {
        if (mode == Mode.AutoShell && aTh1 != null) { theta1Deg = aTh1[aIdx]; theta2Deg = aTh2[aIdx]; d3 = aD3[aIdx]; aIdx = (aIdx + 1) % aTh1.Length; return; }
        float dt = Time.deltaTime;
        theta1Deg += w1 * dt; theta2Deg += w2 * dt; d3 += v3f * dt;
        if (theta1Deg < theta1Min || theta1Deg > theta1Max) { w1 = -w1; theta1Deg = Mathf.Clamp(theta1Deg, theta1Min, theta1Max); }
        if (theta2Deg < theta2Min || theta2Deg > theta2Max) { w2 = -w2; theta2Deg = Mathf.Clamp(theta2Deg, theta2Min, theta2Max); }
        if (d3 < d3Min || d3 > d3Max) { v3f = -v3f; d3 = Mathf.Clamp(d3, d3Min, d3Max); }
    }

    // ═══ TRAIL ═══
    void DoTrail()
    {
        if (!trail || !eeTip) return;
        if (trailFrameSkip > 1 && (++trailFrame % trailFrameSkip) != 0) return;

        var p = eeTip.position;
        if (trailHasLast && (p - trailLast).sqrMagnitude < trailMinDistSqr) return;

        trailIdx = (trailIdx + 1) % maxTrailPoints;
        trail.SetPosition(trailIdx, p);
        trailLast = p;
        trailHasLast = true;
    }

    void ClrTrail()
    {
        if (!trail || !eeTip) return;
        trailIdx = 0;
        var p = eeTip.position;
        for (int i = 0; i < maxTrailPoints; i++) trail.SetPosition(i, p);
        trailLast = p;
        trailHasLast = true;
    }

    // ══════════════════════════════════════════════════════════════
    //  ROBOT — HEAVY, GROUNDED, FUTURISTIC INDUSTRIAL DESIGN
    //  The arm sweeps upper hemisphere only → column can be thick.
    // ══════════════════════════════════════════════════════════════
    void BuildRobot()
    {
        // ── BASE PLATFORM ──────────────────────────────────
        // 3-layer sandwich: rubber pad → main body → top plate
        Cy("RubberPad", transform, V(0, 0.003f, 0), V(0.72f, 0.003f, 0.72f), RUBBER);
        Bx("BaseBody",  transform, V(0, 0.028f, 0), V(0.80f, 0.044f, 0.60f), DARK_STEEL);
        Bx("BaseTop",   transform, V(0, 0.053f, 0), V(0.78f, 0.006f, 0.58f), MID_STEEL);

        // front accent panel
        Bx("PnlF", transform, V(0, 0.028f, -0.304f), V(0.60f, 0.036f, 0.008f), CYAN_A);
        // side accent strips
        Bx("StL", transform, V(-0.404f, 0.028f, 0), V(0.008f, 0.036f, 0.44f), CYAN_DIM);
        Bx("StR", transform, V(0.404f, 0.028f, 0), V(0.008f, 0.036f, 0.44f), CYAN_DIM);

        // corner radius blocks
        for (int i = 0; i < 4; i++)
        {
            float sx = (i % 2 == 0 ? -1 : 1) * 0.38f, sz = (i < 2 ? -1 : 1) * 0.28f;
            var cn = Bx($"Cn{i}", transform, V(sx, 0.028f, sz), V(0.05f, 0.044f, 0.05f), MID_STEEL);
            cn.transform.localRotation = Quaternion.Euler(0, 45, 0);
        }

        // hazard stripes (front face)
        for (int i = -5; i <= 5; i++)
            Bx($"Hz{i}", transform, V(i * 0.052f, 0.053f, -0.284f), V(0.042f, 0.004f, 0.014f), i % 2 == 0 ? YELLOW : RUBBER);

        // mounting feet (heavy rubber + chrome cap)
        for (int i = 0; i < 4; i++)
        {
            float fx = (i % 2 == 0 ? -1 : 1) * 0.34f, fz = (i < 2 ? -1 : 1) * 0.25f;
            Cy($"FtR{i}", transform, V(fx, 0.003f, fz), V(0.050f, 0.003f, 0.050f), RUBBER);
            Cy($"FtC{i}", transform, V(fx, 0.007f, fz), V(0.036f, 0.001f, 0.036f), CHROME);
        }

        // status LEDs
        Sp("LPwr", transform, V(-0.26f, 0.058f, -0.275f), V(0.010f, 0.006f, 0.010f), C(30, 240, 80));
        Sp("LErr", transform, V(-0.24f, 0.058f, -0.275f), V(0.010f, 0.006f, 0.010f), RED_S);
        Sp("LCom", transform, V(-0.22f, 0.058f, -0.275f), V(0.010f, 0.006f, 0.010f), CYAN_G);

        // nameplate
        Bx("NP", transform, V(0.24f, 0.058f, -0.280f), V(0.12f, 0.003f, 0.018f), WHITE_L);
        Bx("NPT", transform, V(0.24f, 0.059f, -0.280f), V(0.10f, 0.002f, 0.010f), NAVY);

        // ── TURNTABLE (θ1 axis = +Y) ──────────────────────
        float ttY = 0.056f;
        ttPiv = Em("TT", transform, V(0, ttY, 0));

        // bearing ring → turntable layers
        Cy("BRng", ttPiv, V(0, -0.002f, 0), V(0.54f, 0.002f, 0.54f), RUBBER);
        Cy("TTlo", ttPiv, V(0, 0.010f, 0), V(0.50f, 0.016f, 0.50f), STEEL);
        Cy("TThi", ttPiv, V(0, 0.022f, 0), V(0.44f, 0.008f, 0.44f), LIGHT_STEEL);
        Cy("TTcap",ttPiv, V(0, 0.028f, 0), V(0.38f, 0.004f, 0.38f), SILVER);

        // encoder ring
        Cy("EncR", ttPiv, V(0, 0.030f, 0), V(0.48f, 0.001f, 0.48f), CYAN_A);
        for (int i = 0; i < 36; i++)
        {
            float a = i * 10f * Mathf.Deg2Rad, r = 0.225f;
            bool mj = i % 9 == 0;
            Bx($"ET{i}", ttPiv, V(Mathf.Cos(a) * r, 0.031f, Mathf.Sin(a) * r),
                V(mj ? 0.008f : 0.003f, 0.002f, 0.002f), mj ? CYAN_G : LIGHT_STEEL);
        }

        // hub bolts
        for (int i = 0; i < 8; i++)
        {
            float a = i * 45f * Mathf.Deg2Rad, r = 0.17f;
            Cy($"HB{i}", ttPiv, V(Mathf.Cos(a) * r, 0.029f, Mathf.Sin(a) * r), V(0.012f, 0.002f, 0.012f), CHROME);
        }

        // ── COLUMN / BODY — THICK & SUBSTANTIAL ───────────
        // The arm NEVER points below horizontal, so this can be wide.
        float colR = 0.09f;

        // lower column section (wide, robust)
        Cy("CL1", ttPiv, V(0, 0.052f, 0), V(colR * 2 + 0.06f, 0.030f, colR * 2 + 0.06f), DARK_STEEL);
        Cy("CR1", ttPiv, V(0, 0.068f, 0), V(colR * 2 + 0.08f, 0.003f, colR * 2 + 0.08f), CYAN_A);

        // main body (tall, heavy cylinder)
        Cy("CMain", ttPiv, V(0, 0.115f, 0), V(colR * 2 + 0.04f, 0.050f, colR * 2 + 0.04f), MID_STEEL);

        // accent ring mid
        Cy("CRm", ttPiv, V(0, 0.135f, 0), V(colR * 2 + 0.06f, 0.003f, colR * 2 + 0.06f), CYAN_DIM);

        // upper taper section
        Cy("CUp", ttPiv, V(0, 0.178f, 0), V(colR * 2, 0.030f, colR * 2), STEEL);

        // vent slots (decorative, front and back)
        for (int i = -1; i <= 1; i++)
        {
            Bx($"VF{i}", ttPiv, V(i * 0.04f, 0.115f, -(colR + 0.025f)), V(0.025f, 0.032f, 0.004f), RUBBER);
            Bx($"VB{i}", ttPiv, V(i * 0.04f, 0.115f, (colR + 0.025f)), V(0.025f, 0.032f, 0.004f), RUBBER);
        }

        // cable conduit
        Bx("CblC", ttPiv, V(colR + 0.015f, 0.12f, 0), V(0.014f, 0.10f, 0.014f), RUBBER);
        for (int i = 0; i < 4; i++)
            Bx($"CC{i}", ttPiv, V(colR + 0.015f, 0.06f + i * 0.030f, 0), V(0.020f, 0.004f, 0.020f), MID_STEEL);

        // dome cap on column (hemisphere, transitions to shoulder)
        Sp("Dome", ttPiv, V(0, 0.200f, 0), V(colR * 2 + 0.02f, 0.06f, colR * 2 + 0.02f), STEEL);
        Cy("DomeR", ttPiv, V(0, 0.195f, 0), V(colR * 2 + 0.04f, 0.003f, colR * 2 + 0.04f), CYAN_A);

        // ── SHOULDER JOINT (θ2) ────────────────────────────
        // Sits on top of the dome. Arm sweeps in upper hemisphere.
        float shY = d1 - ttY;
        shPiv = Em("SH", ttPiv, V(0, shY, 0));

        // transition collar
        Cy("ShCol", shPiv, V(0, -0.020f, 0), V(0.14f, 0.020f, 0.14f), LIGHT_STEEL);
        Cy("ShClR", shPiv, V(0, -0.008f, 0), V(0.16f, 0.003f, 0.16f), CYAN_A);

        // main joint ball (KUKA orange — the signature element)
        Sp("JBall", shPiv, V(0, 0, 0), V(0.17f, 0.17f, 0.17f), ORANGE);

        // bearing seals
        Cy("BSeal", shPiv, V(0, 0, 0), V(0.19f, 0.004f, 0.19f), RUBBER);
        Cy("BRing", shPiv, V(0, 0, 0), V(0.15f, 0.006f, 0.15f), ORANGE_D);

        // U-yoke bearing housing (open sides for arm sweep)
        Bx("YkL", shPiv, V(0, 0, 0.090f), V(0.12f, 0.15f, 0.020f), SILVER);
        Bx("YkR", shPiv, V(0, 0, -0.090f), V(0.12f, 0.15f, 0.020f), SILVER);

        // yoke bridge (top)
        Bx("YkB", shPiv, V(0, 0.072f, 0), V(0.07f, 0.012f, 0.200f), LIGHT_STEEL);

        // reinforcement ribs on yoke
        Bx("RbL", shPiv, V(0, 0.058f, 0.082f), V(0.08f, 0.010f, 0.006f), CHROME);
        Bx("RbR", shPiv, V(0, 0.058f, -0.082f), V(0.08f, 0.010f, 0.006f), CHROME);

        // pivot bolts + washers
        Cy("PBL", shPiv, V(0, 0, 0.104f), V(0.024f, 0.004f, 0.024f), CHROME);
        Cy("PBR", shPiv, V(0, 0, -0.104f), V(0.024f, 0.004f, 0.024f), CHROME);
        Cy("PWL", shPiv, V(0, 0, 0.102f), V(0.032f, 0.002f, 0.032f), LIGHT_STEEL);
        Cy("PWR", shPiv, V(0, 0, -0.102f), V(0.032f, 0.002f, 0.032f), LIGHT_STEEL);

        // grease fitting
        Sp("Grs", shPiv, V(0.070f, 0.058f, 0), V(0.010f, 0.010f, 0.010f), CHROME);

        // ── ARM BEAM (dynamic stretch) ─────────────────────
        float hw = 0.038f, hh = 0.036f;
        float bL0 = 0.22f, bM0 = BEAM_S + bL0 * 0.5f;

        beam = Bx("Beam", shPiv, V(bM0, 0, 0), V(bL0, hh * 2, hw * 2), GREEN_A).transform;
        rT = Bx("RT", shPiv, V(bM0, hh + 0.003f, 0), V(bL0, 0.006f, hw * 1.2f), CHROME).transform;
        rB = Bx("RB", shPiv, V(bM0, -(hh + 0.003f), 0), V(bL0, 0.006f, hw * 1.2f), CHROME).transform;
        rL = Bx("RL", shPiv, V(bM0, 0, hw + 0.003f), V(bL0, hh * 1.2f, 0.006f), CHROME).transform;
        rR = Bx("RR", shPiv, V(bM0, 0, -(hw + 0.003f)), V(bL0, hh * 1.2f, 0.006f), CHROME).transform;
        bCap = Bx("BCap", shPiv, V(bL0 + BEAM_S + 0.008f, 0, 0), V(0.010f, hh * 2.2f, hw * 2.2f), LIGHT_STEEL).transform;

        // accent stripes
        Bx("AS1", shPiv, V(bM0, hh + 0.007f, 0), V(bL0 * 0.6f, 0.003f, hw * 0.5f), GREEN_D);
        Bx("AS2", shPiv, V(bM0, -(hh + 0.007f), 0), V(bL0 * 0.6f, 0.003f, hw * 0.5f), GREEN_D);

        // inner telescope rod
        bInner = Bx("Tel", shPiv, V(BEAM_S + 0.08f, 0, 0), V(0.16f, hh * 0.68f, hw * 0.68f), PURPLE).transform;

        // ── SLIDER CARRIAGE ────────────────────────────────
        slPiv = Em("SL", shPiv, V(d3, 0, 0));
        float sw = 0.042f, sh2 = 0.028f;

        Bx("Car", slPiv, V(0, 0, 0), V(0.10f, sh2 * 2.5f, sw * 2.5f), PURPLE);
        // face plates
        Bx("FpF", slPiv, V(0.054f, 0, 0), V(0.004f, sh2 * 2.8f, sw * 2.8f), LIGHT_STEEL);
        Bx("FpB", slPiv, V(-0.054f, 0, 0), V(0.004f, sh2 * 2.8f, sw * 2.8f), LIGHT_STEEL);
        // wiper seals
        Bx("WsF", slPiv, V(0.058f, 0, 0), V(0.002f, sh2 * 3f, sw * 3f), RUBBER);
        Bx("WsB", slPiv, V(-0.058f, 0, 0), V(0.002f, sh2 * 3f, sw * 3f), RUBBER);

        // carriage bolts
        for (int c = 0; c < 4; c++)
            Cy($"CBt{c}", slPiv, V(0.054f, (c < 2 ? 1 : -1) * sh2 * 0.85f, (c % 2 == 0 ? 1 : -1) * sw * 0.85f),
                V(0.008f, 0.002f, 0.008f), CHROME);

        // connecting shaft (rotated 90° to lie along X)
        Cy("Shf", slPiv, V(0.088f, 0, 0), V(0.018f, 0.048f, 0.018f), CHROME);
        slPiv.Find("Shf").localRotation = Quaternion.Euler(0, 0, 90);

        // proximity sensor
        Bx("Prx", slPiv, V(0, sh2 * 1.5f, 0), V(0.018f, 0.007f, 0.013f), DARK_STEEL);
        Sp("PrxL", slPiv, V(0, sh2 * 1.5f + 0.004f, 0), V(0.006f, 0.004f, 0.006f), CYAN_G);

        // purple accent on carriage
        Bx("CAcc", slPiv, V(0, 0, sw * 1.3f), V(0.06f, sh2 * 1.5f, 0.003f), PURPLE_D);
        Bx("CAc2", slPiv, V(0, 0, -(sw * 1.3f)), V(0.06f, sh2 * 1.5f, 0.003f), PURPLE_D);

        // ── GRIPPER / END EFFECTOR ─────────────────────────
        float gs = 0.11f, pl = 0.048f;
        Bx("GBs", slPiv, V(gs, 0, 0), V(0.018f, 0.048f, 0.048f), LIGHT_STEEL);
        Bx("GPm", slPiv, V(gs + pl * 0.5f + 0.010f, 0, 0), V(pl, 0.036f, 0.026f), RED_S);
        Bx("GSt", slPiv, V(gs + pl * 0.5f + 0.010f, 0.019f, 0), V(pl * 0.7f, 0.003f, 0.020f), YELLOW);

        float fb = gs + pl + 0.012f, fl = 0.062f, gap = 0.013f;
        // finger guides
        Bx("FgL", slPiv, V(fb - 0.003f, 0, gap + 0.002f), V(0.010f, 0.024f, 0.004f), LIGHT_STEEL);
        Bx("FgR", slPiv, V(fb - 0.003f, 0, -(gap + 0.002f)), V(0.010f, 0.024f, 0.004f), LIGHT_STEEL);
        // fingers
        Bx("FnL", slPiv, V(fb + fl * 0.5f, 0, gap), V(fl, 0.013f, 0.010f), ORANGE_D);
        Bx("FnR", slPiv, V(fb + fl * 0.5f, 0, -gap), V(fl, 0.013f, 0.010f), ORANGE_D);
        // tips
        Bx("TpL", slPiv, V(fb + fl + 0.006f, 0, gap * 0.78f), V(0.012f, 0.013f, 0.007f), ORANGE_D);
        Bx("TpR", slPiv, V(fb + fl + 0.006f, 0, -gap * 0.78f), V(0.012f, 0.013f, 0.007f), ORANGE_D);
        // rubber pads
        Bx("PdL", slPiv, V(fb + fl + 0.006f, 0, gap * 0.50f), V(0.008f, 0.010f, 0.003f), RUBBER);
        Bx("PdR", slPiv, V(fb + fl + 0.006f, 0, -gap * 0.50f), V(0.008f, 0.010f, 0.003f), RUBBER);
        // finger pins
        Cy("PnL", slPiv, V(fb, 0, gap + 0.007f), V(0.005f, 0.003f, 0.005f), CHROME);
        Cy("PnR", slPiv, V(fb, 0, -(gap + 0.007f)), V(0.005f, 0.003f, 0.005f), CHROME);

        // end-effector tip point
        var ee = new GameObject("EE"); ee.transform.SetParent(slPiv, false);
        ee.transform.localPosition = V(fb + fl + 0.013f, 0, 0); eeTip = ee.transform;
    }

    // ═══ ENVIRONMENT ═══
    void SetupCam()
    {
        if (!cam) { cam = Camera.main; if (!cam) cam = FindObjectOfType<Camera>(); }
        var c = cam; if (!c) return;
        c.backgroundColor = new Color(0.025f, 0.03f, 0.06f);
        c.clearFlags = CameraClearFlags.SolidColor;
        UpdateCamNow();
    }

    void UpdateCamNow()
    {
        if (!cam) { cam = Camera.main; if (!cam) cam = FindObjectOfType<Camera>(); }
        if (!cam) return;
        float pr = camPitch * Mathf.Deg2Rad, yr = camYaw * Mathf.Deg2Rad;
        cam.transform.position = camTarget + new Vector3(
            Mathf.Sin(yr) * Mathf.Cos(pr),
            Mathf.Sin(pr),
            Mathf.Cos(yr) * Mathf.Cos(pr)
        ) * camDist;
        cam.transform.LookAt(camTarget);
    }

    void SetupTrail()
    {
        var go = new GameObject("Trail"); go.transform.SetParent(transform);
        trail = go.AddComponent<LineRenderer>();
        trail.positionCount = maxTrailPoints; trail.startWidth = 0.006f; trail.endWidth = 0.006f;
        trail.startColor = TEAL_TRAIL; trail.endColor = TEAL_TRAIL;
        trail.numCapVertices = 4; trail.numCornerVertices = 4;
        trail.material = new Material(Shader.Find("Sprites/Default")) { color = TEAL_TRAIL };
        trail.useWorldSpace = true;
        var p = eeTip ? eeTip.position : Vector3.zero;
        for (int i = 0; i < maxTrailPoints; i++) trail.SetPosition(i, p);
        trailLast = p;
        trailHasLast = true;
    }

    void BuildEnv()
    {
        var g = GameObject.CreatePrimitive(PrimitiveType.Plane);
        g.name = "Floor"; g.transform.SetParent(transform.parent);
        g.transform.position = Vector3.zero; g.transform.localScale = V(3, 1, 3);
        Mt(g, new Color(0.07f, 0.08f, 0.11f), 0.92f, 0.08f);
        StripCollider(g);

        // grid
        int gridN = Mobile ? 6 : 8;
        for (int i = -gridN; i <= gridN; i++)
        {
            float p = i * 0.3f; bool m = i % 4 == 0;
            var gc = m ? new Color(0.16f, 0.17f, 0.20f) : new Color(0.10f, 0.11f, 0.13f);
            Bx($"GX{i}", g.transform, V(p, 0.0003f, 0), V(m ? 0.003f : 0.001f, 0.001f, 6), gc);
            Bx($"GZ{i}", g.transform, V(0, 0.0003f, p), V(6, 0.001f, m ? 0.003f : 0.001f), gc);
        }

        // safety ring
        int ringN = Mobile ? 24 : 48;
        for (int i = 0; i < ringN; i++)
        {
            float a = i * (360f / ringN) * Mathf.Deg2Rad;
            Bx($"SR{i}", g.transform, V(Mathf.Cos(a) * 0.12f, 0.001f, Mathf.Sin(a) * 0.12f),
                V(0.010f, 0.001f, 0.010f), YELLOW);
        }

        // 3-point lighting
        MkL("Key", V(2, 3, -1.5f), LightType.Directional, new Color(1, 0.95f, 0.90f), 1.05f);
        MkL("Fill", V(-2, 1.5f, 1), LightType.Point, new Color(0.25f, 0.45f, 0.75f), 0.55f, 9);
        MkL("Rim", V(-0.5f, 2, 2.5f), LightType.Point, new Color(0, 0.55f, 0.75f), 0.35f, 6);
    }

    void MkL(string n, Vector3 p, LightType t, Color c, float i, float r = 0)
    {
        var go = new GameObject(n); go.transform.SetParent(transform.parent); go.transform.position = p;
        var l = go.AddComponent<Light>(); l.type = t; l.color = c; l.intensity = i;
        if (r > 0) l.range = r; if (t == LightType.Directional) go.transform.rotation = Quaternion.LookRotation(-p);
    }

    // ═══ GUI ═══
    Texture2D _wt;
    Texture2D Wt { get { if (!_wt) { _wt = new Texture2D(1, 1); _wt.SetPixel(0, 0, Color.white); _wt.Apply(); } return _wt; } }

    void OnGUI()
    {
        EnsureGuiStyles();
        if (mode == Mode.Menu) DrawMenu();
        else { DrawHUD(); if (mode == Mode.Manual && Mobile) DrawTouch(); if (mode != Mode.Manual) DrawAutoUI(); }
    }

    void EnsureGuiStyles()
    {
        float s = Sc;
        if (styTitle != null && Mathf.Abs(guiScaleCached - s) < 0.01f) return;
        guiScaleCached = s;

        styTitle = St(R(22 * s), FontStyle.Bold, TextAnchor.MiddleCenter, CYAN_G);
        stySub   = St(R(11 * s), FontStyle.Normal, TextAnchor.MiddleCenter, SILVER);
        styHudT  = St(R(13 * s), FontStyle.Bold, TextAnchor.UpperLeft, CYAN_G);
        styHudL  = St(R(11 * s), FontStyle.Normal, TextAnchor.UpperLeft, Color.white);
        styHudD  = St(R(10 * s), FontStyle.Normal, TextAnchor.UpperLeft, SILVER);

        btnMain  = new GUIStyle(GUI.skin.button) { fontSize = R(15 * s), fontStyle = FontStyle.Bold };
        btnTouch = new GUIStyle(GUI.skin.button) { fontSize = R(16 * s), fontStyle = FontStyle.Bold };
        btnSmall = new GUIStyle(GUI.skin.button) { fontSize = R(12 * s), fontStyle = FontStyle.Bold };
        btnPause = new GUIStyle(GUI.skin.button) { fontSize = R(14 * s), fontStyle = FontStyle.Bold };
    }

    void DrawMenu()
    {
        float s = Sc;
        float bw = Mathf.Min(380 * s, Screen.width * 0.92f), bh = 52 * s, gp = 10 * s;
        float cx = Screen.width * 0.5f, cy = Screen.height * 0.5f;

        float th = 38 * s + bh * 5 + gp * 7 + 48 * s, y = cy - th * 0.5f;
        GUI.Box(new Rect(cx - bw * 0.5f - 18 * s, y - 10 * s, bw + 36 * s, th + 20 * s), "");
        y += 4 * s;
        GUI.Label(new Rect(cx - bw * 0.5f, y, bw, 32 * s), "SPHERICAL MANIPULATOR", styTitle); y += 34 * s;
        GUI.Label(new Rect(cx - bw * 0.5f, y, bw, 18 * s), "3-DOF  R-R-P  │  DH Convention", stySub); y += 20 * s;
        GUI.Label(new Rect(cx - bw * 0.5f, y, bw, 16 * s), $"d1={d1}m  θ1∈[±180°]  θ2∈[±90°]  d3∈[{d3Min}–{d3Max}m]", stySub); y += 22 * s;

        if (GUI.Button(new Rect(cx - bw * 0.5f, y, bw, bh), Mobile ? "Manual Control  (Touch)" : "Manual  (A/D  W/S  Q/E)", btnMain)) { mode = Mode.Manual; ClrTrail(); } y += bh + gp;
        if (GUI.Button(new Rect(cx - bw * 0.5f, y, bw, bh), "Auto: Spherical Shell", btnMain)) { mode = Mode.AutoShell; MkShell(); ClrTrail(); paused = false; } y += bh + gp;
        if (GUI.Button(new Rect(cx - bw * 0.5f, y, bw, bh), "Auto: 3-DOF Exploration", btnMain)) { mode = Mode.Auto3DOF; Mk3(); ClrTrail(); paused = false; } y += bh + gp;
        GUI.Label(new Rect(cx - bw * 0.5f, y, bw, 16 * s), "Single Joint:", stySub); y += 18 * s;
        float bw3 = (bw - gp * 2) / 3f;
        if (GUI.Button(new Rect(cx - bw * 0.5f, y, bw3, bh), "θ1", btnMain)) { mode = Mode.AutoSingle; Mk1(0); ClrTrail(); paused = false; }
        if (GUI.Button(new Rect(cx - bw * 0.5f + bw3 + gp, y, bw3, bh), "θ2", btnMain)) { mode = Mode.AutoSingle; Mk1(1); ClrTrail(); paused = false; }
        if (GUI.Button(new Rect(cx - bw * 0.5f + (bw3 + gp) * 2, y, bw3, bh), "d3", btnMain)) { mode = Mode.AutoSingle; Mk1(2); ClrTrail(); paused = false; }

        // Exit button (for desktop / Android)
        float exW = 80 * s, exH = 32 * s;
        if (GUI.Button(new Rect(Screen.width - exW - 18 * s, Screen.height - exH - 18 * s, exW, exH), "EXIT", btnSmall))
        {
            Application.Quit();
        }
    }

    void DrawHUD()
    {
        float s = Sc;
        float x = 8 * s, y = 8 * s, w = 270 * s, h = 17 * s;
        GUI.Box(new Rect(x, y, w, 165 * s), ""); x += 6 * s; y += 4 * s;
        string ms = mode == Mode.Manual ? "MANUAL" : mode == Mode.AutoShell ? "SHELL" : mode == Mode.Auto3DOF ? "3-DOF" : "SINGLE";
        GUI.Label(new Rect(x, y, w, h), $"R-R-P DH │ {ms}", styHudT); y += h + 2 * s;
        GUI.Label(new Rect(x, y, w, h), $"θ1={theta1Deg,7:F1}°  θ2={theta2Deg,6:F1}°", styHudL); y += h;
        GUI.Label(new Rect(x, y, w, h), $"d3={d3:F3}m  d1={d1:F3}m", styHudL); y += h;
        GUI.Label(new Rect(x, y, w, h), $"EE({eePosDH.x:F3},{eePosDH.y:F3},{eePosDH.z:F3})", styHudD); y += h;
        if (eeTip) { var p = eeTip.position; GUI.Label(new Rect(x, y, w, h), $"Unity({p.x:F3},{p.y:F3},{p.z:F3})", styHudD); } y += h + 4 * s;
        if (mode == Mode.Manual) GUI.Label(new Rect(x, y, w, h), Mobile ? "Joystick → θ1/θ2 | Buttons → d3" : "A/D→θ1  W/S→θ2  Q/E→d3", styHudD);
        else GUI.Label(new Rect(x, y, w, h), paused ? "▐▐ PAUSED" : "▶ RUNNING", styHudL);
    }

    void DrawTouch()
    {
        float s = Sc, pad = 18 * s, bw = 80 * s, bht = 60 * s;
        // joystick visual
        float jR = 65 * s, jCx = pad + jR + 8 * s, jCy = Screen.height - pad - jR - 8 * s;
        DCirc(jCx, jCy, jR, new Color(1, 1, 1, 0.10f));
        DCirc(jCx + joyDelta.x * jR * 0.7f, jCy - joyDelta.y * jR * 0.7f, jR * 0.32f, new Color(0, 0.85f, 1, 0.45f));
        GUI.Label(new Rect(jCx - jR, jCy - jR - 16 * s, jR * 2, 14 * s), "θ1 / θ2", styHudD);

        // d3 buttons
        float rx = Screen.width - pad - bw, by = Screen.height - pad - bht;
        touchExt = GUI.RepeatButton(new Rect(rx, by - bht - 6 * s, bw, bht), "d3 +", btnTouch);
        touchRet = GUI.RepeatButton(new Rect(rx, by, bw, bht), "d3 −", btnTouch);

        // menu button
        if (GUI.Button(new Rect(Screen.width - 65 * s - pad, pad, 65 * s, 36 * s), "MENU", btnSmall)) mode = Mode.Menu;
    }

    void DrawAutoUI()
    {
        float s = Sc, pad = 18 * s;
        float bw = 105 * s, bh = 44 * s;
        if (GUI.Button(new Rect(Screen.width * 0.5f - bw * 0.5f, Screen.height - pad - bh, bw, bh), paused ? "▶ PLAY" : "▐▐ PAUSE", btnPause)) paused = !paused;
        if (GUI.Button(new Rect(Screen.width - 65 * s - pad, pad, 65 * s, 36 * s), "MENU", btnSmall)) mode = Mode.Menu;
    }

    void DCirc(float cx, float cy, float r, Color c)
    { GUI.color = c; GUI.DrawTexture(new Rect(cx - r, cy - r, r * 2, r * 2), Wt, ScaleMode.StretchToFill, true, 1, c, 0, r); GUI.color = Color.white; }

    GUIStyle St(int sz, FontStyle fs, TextAnchor a, Color c)
    { var s = new GUIStyle(GUI.skin.label) { fontSize = sz, fontStyle = fs, alignment = a }; s.normal.textColor = c; return s; }
    int R(float v) => Mathf.RoundToInt(v);

    // ═══ FACTORIES ═══
    static Vector3 V(float x, float y, float z) => new Vector3(x, y, z);
    static float[] Lin(float a, float b, int n) { var r = new float[n]; for (int i = 0; i < n; i++) r[i] = a + (b - a) * i / (n - 1); return r; }
    Transform Em(string n, Transform p, Vector3 l) { var g = new GameObject(n); g.transform.SetParent(p, false); g.transform.localPosition = l; return g.transform; }
    GameObject Bx(string n, Transform p, Vector3 l, Vector3 s, Color c)
    { var g = GameObject.CreatePrimitive(PrimitiveType.Cube); g.name = n; g.transform.SetParent(p, false); g.transform.localPosition = l; g.transform.localScale = s; StripCollider(g); Mt(g, c); return g; }
    GameObject Cy(string n, Transform p, Vector3 l, Vector3 s, Color c)
    { var g = GameObject.CreatePrimitive(PrimitiveType.Cylinder); g.name = n; g.transform.SetParent(p, false); g.transform.localPosition = l; g.transform.localScale = s; StripCollider(g); Mt(g, c); return g; }
    GameObject Sp(string n, Transform p, Vector3 l, Vector3 s, Color c)
    { var g = GameObject.CreatePrimitive(PrimitiveType.Sphere); g.name = n; g.transform.SetParent(p, false); g.transform.localPosition = l; g.transform.localScale = s; StripCollider(g); Mt(g, c); return g; }

    void StripCollider(GameObject go)
    {
        var col = go.GetComponent<Collider>();
        if (col) Destroy(col);
    }

    void Mt(GameObject go, Color c, float sm = 0.78f, float mt = 0.32f)
    {
        var r = go.GetComponent<Renderer>(); if (!r) return;

        Material m;
        if (baseMaterial)
        {
            // Use user-assigned material so its shader is guaranteed included in builds.
            m = new Material(baseMaterial);
        }
        else
        {
            // Prefer URP Lit (project default). Fallback to Standard if needed.
            Shader sh = Shader.Find("Universal Render Pipeline/Lit");
            if (!sh) sh = Shader.Find("Standard");
            if (!sh) return;
            m = new Material(sh);
        }
        // URP uses _BaseColor, Standard uses _Color – set both so either works.
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
        if (m.HasProperty("_Color"))     m.SetColor("_Color", c);
        if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", sm);
        if (m.HasProperty("_Glossiness")) m.SetFloat("_Glossiness", sm);
        if (m.HasProperty("_Metallic"))   m.SetFloat("_Metallic", mt);

        r.material = m;
    }
}
