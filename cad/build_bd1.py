import adsk.core, adsk.fusion, math

MM = 0.1
tbm = adsk.fusion.TemporaryBRepManager.get()
UNION = adsk.fusion.BooleanTypes.UnionBooleanType
DIFF = adsk.fusion.BooleanTypes.DifferenceBooleanType

# ---------- primitives (mm) ----------
def P(x, y, z): return adsk.core.Point3D.create(x * MM, y * MM, z * MM)
def V(x, y, z): return adsk.core.Vector3D.create(x, y, z)

def box(x0, x1, y0, y1, z0, z1):
    a, b = sorted((x0, x1)); c, d = sorted((y0, y1)); e, f = sorted((z0, z1))
    obb = adsk.core.OrientedBoundingBox3D.create(P((a + b) / 2, (c + d) / 2, (e + f) / 2), V(1, 0, 0), V(0, 1, 0), (b - a) * MM, (d - c) * MM, (f - e) * MM)
    return tbm.createBox(obb)

def cyl(axis, c1, c2, r, a0, a1):
    if axis == 'x': p0, p1 = P(a0, c1, c2), P(a1, c1, c2)
    elif axis == 'y': p0, p1 = P(c1, a0, c2), P(c1, a1, c2)
    else: p0, p1 = P(c1, c2, a0), P(c1, c2, a1)
    return tbm.createCylinderOrCone(p0, r * MM, p1, r * MM)

def U(t, *tools):
    for x in tools:
        assert tbm.booleanOperation(t, x, UNION), 'union failed'
    return t

def C(t, *tools):
    for x in tools:
        assert tbm.booleanOperation(t, x, DIFF), 'cut failed'
    return t

# ---------- 4x4 math (mm) ----------
def T(x, y, z): return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]
def Rx(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]
def Ry(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]
def mul(*ms):
    r = ms[0]
    for m in ms[1:]:
        r = [[sum(r[i][k] * m[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
    return r
def inv(A):
    R = [[A[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(R[i][k] * A[k][3] for k in range(3)) for i in range(3)]
    return [R[0] + [t[0]], R[1] + [t[1]], R[2] + [t[2]], [0, 0, 0, 1]]
def F(A):
    m = adsk.core.Matrix3D.create()
    arr = []
    for i in range(4):
        for j in range(4):
            arr.append(A[i][j] * (MM if (j == 3 and i < 3) else 1))
    m.setWithArray(arr)
    return m

def X(s, a, b): return (s * a, s * b)

# ---------- parts (robot frame: x left+, y up, z forward) ----------
def pelvis():
    b = box(-18, 18, -25, 25, -35, 35)
    C(b, box(-16, 16, -22.5, 25.1, -33, 33))
    for s in (1, -1):
        U(b, cyl('x', 0, 0, 12, s * 18, s * 24.5))
        C(b, cyl('x', 0, 0, 2.5, s * 15, s * 25),
          cyl('x', 0, 0, 3.7, s * 22.5, s * 24.6),
          box(*X(s, 22.5, 24.6), 0, 17, -2.3, 2.3),
          box(*X(s, 15.5, 18.5), 13, 21, -30, -18))
    for (y, z) in ((-2.8, 27.95), (-2.8, -27.95), (16.2, 27.95), (16.2, -27.95)):
        U(b, cyl('x', y, z, 2.6, -16.1, -12))
        C(b, cyl('x', y, z, 0.9, -17, -11.9))
    for x in (7.6, -7.6):
        U(b, cyl('z', x, 18.9, 2.6, 29, 33.1))
        C(b, cyl('z', x, 18.9, 0.9, 28.9, 34))
    for z0, z1 in ((17, 23), (-23, -17)):
        U(b, box(10, 16.1, 17, 25, z0, z1), box(-16.1, -12.5, 17, 25, z0, z1))
    for z in (20, -20):
        C(b, cyl('y', 13, z, 0.9, 16, 25.1), cyl('y', -14.25, z, 0.9, 16, 25.1))
    return b

def pelvis_lid():
    b = box(-18, 18, 25, 27.4, -35, 35)
    for z in (20, -20):
        C(b, cyl('y', 13, z, 1.2, 24.9, 27.5), cyl('y', -14.25, z, 1.2, 24.9, 27.5))
    C(b, cyl('y', 0, -27.5, 1.7, 24.9, 27.5), cyl('y', 0, 16.5, 1.7, 24.9, 27.5), box(-4, 4, 24.9, 27.5, -11.5, 0.5))
    return b

def neck():
    b = box(-14, 14, 27.4, 30.4, -31.5, 20.5)
    U(b, box(-10, 10, 30.4, 48.4, -23.5, 12.5))
    C(b, box(-6.3, 6.3, 32.7, 48.5, -17.1, 6.1), box(-3, 3, 27.3, 32.8, -15, 4),
      cyl('y', 0, -19.4, 0.9, 40.4, 48.5), cyl('y', 0, 8.4, 0.9, 40.4, 48.5),
      cyl('y', 0, -27.5, 1.7, 27.3, 30.5), cyl('y', 0, 16.5, 1.7, 27.3, 30.5))
    return b

def yoke():
    b = box(-23, 23, 0, 3, -13, 13)
    for s in (1, -1):
        U(b, box(*X(s, 18.5, 21.5), 3, 19, -7, 7), cyl('x', 19, 0, 7, s * 18.5, s * 21.5))
    C(b, cyl('x', 19, 0, 1.7, -22, -18))
    C(b, cyl('x', 19, 0, 3.7, 18.4, 20.3), box(18.4, 20.3, 4, 19, -2.3, 2.3), cyl('x', 19, 0, 2.5, 18, 22))
    C(b, cyl('y', 0, 0, 3.7, -0.1, 1.8), box(0, 17, -0.1, 1.8, -2.3, 2.3), cyl('y', 0, 0, 2.5, -0.1, 3.1))
    return b

def head_bottom():
    b = box(-36, 36, -12, 33, -58, 58)
    C(b, box(-34, 34, -9.5, 33.1, -56, 56))
    for s in (1, -1):
        C(b, box(*X(s, 17, 23), -12.1, -9.4, -13, 13))
    U(b, box(-3.8, 2.2, -9.5, 6.1, -21.9, -16.9), box(-3.8, 2.2, -9.5, 6.1, 5.9, 10.9))
    C(b, cyl('x', 0, -19.4, 0.9, -3, 2.3), cyl('x', 0, 8.4, 0.9, -3, 2.3))
    U(b, box(-34.1, -31, 5, 8, -27, 51), box(31, 34.1, 5, 8, -27, 51))
    # ESP32 card guides (board vertical at z -31.5..-29.9, x -17.5..34)
    U(b, box(-21.5, -16.5, -9.5, 33, -34, -27.4), box(30, 34.1, -9.5, 10, -34, -27.4))
    C(b, box(-18, -16.4, 4, 33.1, -31.7, -29.7), box(29.9, 34.05, 4, 10.1, -31.7, -29.7))
    # front: eye and lamp
    C(b, cyl('z', 14, 12, 10.2, 55.9, 58.1), cyl('z', -18, 12, 7, 55.9, 58.1))
    # rear: USB-C of charger, switch
    C(b, box(12, 24, -9, -3.5, -58.1, -55.9), box(-17, -11, -4, -1, -58.1, -55.9),
      cyl('z', -21.5, -2.5, 0.9, -58.1, -55.9), cyl('z', -6.5, -2.5, 0.9, -58.1, -55.9))
    # right wall: ESP32 USB
    C(b, box(33.9, 36.1, 14, 22.5, -33, -23))
    # side discs: 3 pilots each side, grille on the left
    for s in (1, -1):
        for (dy, dz) in ((13, 0), (-6.5, -11.26), (-6.5, 11.26)):
            C(b, cyl('x', 12 + dy, -20 + dz, 0.9, s * 33.9, s * 36.1))
    for (dy, dz) in ((0, 0), (3, 0), (-3, 0), (1.5, 2.6), (-1.5, 2.6), (1.5, -2.6), (-1.5, -2.6)):
        C(b, cyl('x', 12 + dy, -20 + dz, 1, -36.1, -33.9))
    # lid bosses: front left/right, rear left
    for (x, z) in ((30, 54), (-30, 54), (-30, -54)):
        U(b, cyl('y', x, z, 3.5, -9.5, 33))
        C(b, cyl('y', x, z, 1.3, 23, 33.1))
    return b

def shelf():
    b = box(-30.6, 30.6, 8, 10, -26.5, 50.5)
    C(b, box(-8, 8, 7.9, 10.1, -14, -11), box(-8, 8, 7.9, 10.1, 30, 33),
      box(-22, -16, 7.9, 10.1, -26.6, -23.4), box(29.5, 30.7, 7.9, 10.1, -26.6, -23.4))
    return b

def head_top():
    b = box(-36, 36, 33, 41, -58, 58)
    C(b, box(-34, 34, 32.9, 38.5, -56, 56))
    U(b, box(-30, 30, 35, 41, 58, 66))
    for (x, z) in ((30, 54), (-30, 54), (-30, -54)):
        C(b, cyl('y', x, z, 1.7, 32.9, 41.1))
    for x in (22, -22):
        U(b, cyl('y', x, -36, 4, 33, 38.6))
        C(b, cyl('y', x, -36, 1.05, 32.9, 41.1))
    return b

def side_disc(grille):
    b = cyl('z', 0, 0, 17, 0, 4)
    r1 = cyl('z', 0, 0, 15, 4, 5); C(r1, cyl('z', 0, 0, 13.5, 3.9, 5.1))
    r2 = cyl('z', 0, 0, 11, 4, 5); C(r2, cyl('z', 0, 0, 9.5, 3.9, 5.1))
    U(b, r1, r2, cyl('z', 0, 0, 6, 4, 7))
    for a in (90, 210, 330):
        C(b, cyl('z', 13 * math.cos(math.radians(a)), 13 * math.sin(math.radians(a)), 1.1, -0.1, 7.1))
    if grille:
        for (x, y) in ((0, 0), (3, 0), (-3, 0), (1.5, 2.6), (-1.5, 2.6), (1.5, -2.6), (-1.5, -2.6)):
            C(b, cyl('z', x, y, 1, -0.1, 7.1))
    return b

def bezel(ro, ri, rl, h):
    b = cyl('z', 0, 0, ro, 0, h)
    C(b, cyl('z', 0, 0, ri, 1, h + 0.1), cyl('z', 0, 0, rl, -0.1, 1.1))
    return b

def antenna_tip():
    b = cyl('z', 0, 0, 2.5, 0, 9)
    C(b, cyl('z', 0, 0, 1.05, 3, 9.1))
    return b

def pivot_cup():
    b = box(-13.2, 13.2, -7.9, 7.9, 0, 8)
    C(b, box(-11.6, 11.6, -6.3, 6.3, 2, 8.1))
    U(b, cyl('z', 5.5, 0, 3.5, -3, 0.5))
    C(b, cyl('z', 5.5, 0, 1.3, -3.1, 2.05))
    return b

def fork_arms(b, s, y_low):
    # inner pivot arm (x 19..22) and outer horn arm (x 59..62) around a joint axis at origin
    U(b, box(*X(s, 19, 22), y_low, 0, -7, 7), cyl('x', 0, 0, 7, s * 19, s * 22),
      box(*X(s, 59, 62), y_low, 0, -7, 7), cyl('x', 0, 0, 7, s * 59, s * 62))
    C(b, cyl('x', 0, 0, 1.7, s * 18.5, s * 22.5))
    C(b, cyl('x', 0, 0, 3.7, s * 58.9, s * 60.8), box(*X(s, 58.9, 60.8), y_low + 0.5, 0, -2.3, 2.3), cyl('x', 0, 0, 2.5, s * 58.5, s * 62.5))
    return b

def servo_housing(b, s, axis_y):
    # lower housing for a servo whose shaft points outward (+s*x) at height axis_y
    U(b, box(*X(s, 22.5, 42.2), axis_y - 11, axis_y + 22.2, -8.1, 8.1))
    C(b, box(*X(s, 26.3, 42.3), axis_y - 6.1, axis_y + 17.1, -6.3, 8.2))
    C(b, cyl('x', axis_y, 0, 1.3, s * 22.4, s * 26.4))
    for dy in (-8.4, 19.4):
        C(b, cyl('x', axis_y + dy, 0, 0.9, s * 36, s * 42.3))
    return b

def thigh(s):
    b = box(*X(s, 41.8, 59.5), -12.6, 23.6, -8.1, 8.1)
    C(b, box(*X(s, 41.7, 57.7), -6.1, 17.1, -6.3, 8.2))
    for y in (-8.4, 19.4):
        C(b, cyl('x', y, 0, 0.9, s * 41.7, s * 48))
    U(b, box(*X(s, 25, 59.5), -17.8, -11.2, -8.1, 8.1))
    servo_housing(b, s, -40)
    return b

def shin(s):
    b = box(*X(s, 19, 62), -18.2, -16.2, -7, 7)
    fork_arms(b, s, -16.2)
    servo_housing(b, s, -40)
    return b

def bracket(s):
    b = box(*X(s, 19, 64), -17.7, -14.7, -16, 16)
    fork_arms(b, s, -14.7)
    U(b, box(*X(s, 29, 66), -31.8, -14.7, -2.3, 0.2))
    C(b, box(*X(s, 35.9, 59.1), -30.1, -17.5, -2.4, 0.3))
    for x in (33.6, 61.4):
        C(b, cyl('z', s * x, -23.8, 0.9, -2.4, 0.3))
    return b

def foot_plate(y0, y1):
    b = box(-12, 12, y0, y1, -14, 12)
    U(b, box(-10, 10, y0, y1, -24, -8), box(-5, 5, y0, y1, -8, 38))
    for s in (1, -1):
        t = box(-4, 4, y0, y1, 0, 30)
        tbm.transform(t, F(mul(T(s * 6, 0, 6), Ry(s * 25))))
        U(b, t)
    return b

def foot():
    b = foot_plate(-16, -13)
    U(b, box(-8, 8, -13, 0, -23.5, -20.5), cyl('z', 0, 0, 8, -23.5, -20.5),
      box(-8, 8, -13, 0, 17.5, 20.5), cyl('z', 0, 0, 8, 17.5, 20.5))
    C(b, cyl('z', 0, 0, 1.7, -24, -20))
    C(b, cyl('z', 0, 0, 3.7, 17.4, 19.3), box(-2.3, 2.3, -12.5, 0, 17.4, 19.3), cyl('z', 0, 0, 2.5, 17, 21))
    return b

def sole():
    return foot_plate(-17.5, -16)

# ---------- placement ----------
TH, TK, TA = -30.0, 60.0, -30.0
BODY = T(0, 109.1, 0)              # Fusion here is Y-up: robot frame = world
YOKE = mul(BODY, T(0, 65.7, 0))
HEAD = mul(BODY, T(0, 84.7, 0))
def leg(s):
    th = mul(BODY, Rx(TH))
    sh = mul(th, T(0, -40, 0), Rx(TK))
    br = mul(sh, T(0, -40, 0), Rx(TA))
    ft = mul(br, T(42 * s, -23.8, 0))
    return th, sh, br, ft
LL, LR = leg(1), leg(-1)

UP = Rx(90); DOWN = Rx(-90); FLAT = T(0, 0, 0); FLIP = Rx(180)

def run(_context: str):
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    assert des.designType == 0
    for i in range(root.occurrences.count - 1, -1, -1):
        root.occurrences.item(i).deleteMe()
    assert root.occurrences.count == 0

    side_x = [70]
    def spare():
        side_x[0] += 30
        return mul(T(side_x[0], 0, 60), Rx(-90))

    parts = [
        ('P01 Таз', pelvis(), UP, [BODY]),
        ('P02 Крышка таза', pelvis_lid(), UP, [BODY]),
        ('P03 Стойка шеи', neck(), UP, [BODY]),
        ('P04 Вилка шеи', yoke(), UP, [YOKE]),
        ('P05 Голова низ', head_bottom(), UP, [HEAD]),
        ('P05b Полка аккумулятора', shelf(), UP, [HEAD]),
        ('P06 Голова верх', head_top(), DOWN, [HEAD]),
        ('P07 Диск левый с решёткой', side_disc(True), FLAT, [mul(HEAD, T(-36, 12, -20), Ry(-90))]),
        ('P07 Диск правый', side_disc(False), FLAT, [mul(HEAD, T(36, 12, -20), Ry(90))]),
        ('P08 Оправа глаза', bezel(14, 10.2, 9, 6), FLAT, [mul(HEAD, T(14, 12, 58))]),
        ('P08 Оправа фонаря', bezel(11, 7, 6, 8), FLAT, [mul(HEAD, T(-18, 12, 58))]),
        ('P09 Бедро левое', thigh(1), FLAT, [LL[0]]),
        ('P09 Бедро правое', thigh(-1), FLAT, [LR[0]]),
        ('P10 Голень левая', shin(1), FLAT, [LL[1]]),
        ('P10 Голень правая', shin(-1), FLAT, [LR[1]]),
        ('P11 Скоба голеностопа левая', bracket(1), Ry(-90), [LL[2]]),
        ('P11 Скоба голеностопа правая', bracket(-1), Ry(90), [LR[2]]),
        ('P12 Стопа', foot(), UP, [LL[3], LR[3]]),
        ('P13 Подошва TPU', sole(), UP, [LL[3], LR[3]]),
        ('P14 Опорная чашка', pivot_cup(), FLIP, [None, None, None]),
        ('P15 Наконечник антенны', antenna_tip(), FLAT, [None, None]),
    ]

    for name, body, R, places in parts:
        pb = tbm.copy(body)
        tbm.transform(pb, F(R))
        bb = pb.boundingBox
        cx = (bb.minPoint.x + bb.maxPoint.x) / 2 / MM
        cy = (bb.minPoint.y + bb.maxPoint.y) / 2 / MM
        mz = bb.minPoint.z / MM
        Pm = mul(T(-cx, -cy, -mz), R)
        tbm.transform(pb, F(T(-cx, -cy, -mz)))
        Pinv = inv(Pm)
        occ = None
        for W in places:
            Mw = mul(W, Pinv) if W is not None else spare()
            if occ is None:
                occ = root.occurrences.addNewComponent(F(Mw))
                comp = occ.component
                comp.name = name
                comp.bRepBodies.add(pb)
                o = occ
            else:
                o = root.occurrences.addExistingComponent(comp, adsk.core.Matrix3D.create())
                o.transform = F(Mw)
            a = o.transform.asArray()
            assert abs(a[3] * 10 - Mw[0][3]) < 0.01 and abs(a[7] * 10 - Mw[1][3]) < 0.01 and abs(a[11] * 10 - Mw[2][3]) < 0.01, 'transform mismatch ' + name
        b2 = comp.bRepBodies.item(0).boundingBox
        dims = [round((getattr(b2.maxPoint, a) - getattr(b2.minPoint, a)) / MM, 1) for a in 'xyz']
        print(name.split(' ')[0], 'x', len(places), '| print size mm', dims)
    # MG90S dummies for clearance checks (not printed)
    sv = box(-11.4, 11.4, -6.1, 6.1, 0, 22.7)
    U(sv, box(-16.1, 16.1, -6.1, 6.1, 15.7, 18.2), cyl('z', 5.5, 0, 5.9, 22.7, 26.7), cyl('z', 5.5, 0, 2.4, 26.7, 31))
    def frame(xd, yd, zd, o):
        return [[xd[0], yd[0], zd[0], o[0]], [xd[1], yd[1], zd[1], o[1]], [xd[2], yd[2], zd[2], o[2]], [0, 0, 0, 1]]
    sp = [mul(BODY, frame((0, 0, 1), (1, 0, 0), (0, 1, 0), (0, 32.7, -5.5))),
          mul(HEAD, frame((0, 0, 1), (0, -1, 0), (1, 0, 0), (-13.5, 0, -5.5)))]
    for s, Lg in ((1, LL), (-1, LR)):
        sp.append(mul(Lg[0], frame((0, -1, 0), (0, 0, s), (-s, 0, 0), (57.5 * s, 5.5, 0))))
        sp.append(mul(Lg[0], frame((0, -1, 0), (0, 0, -s), (s, 0, 0), (26.5 * s, -34.5, 0))))
        sp.append(mul(Lg[1], frame((0, -1, 0), (0, 0, -s), (s, 0, 0), (26.5 * s, -34.5, 0))))
        sp.append(mul(Lg[2], frame((-s, 0, 0), (0, -s, 0), (0, 0, 1), (47.5 * s, -23.8, -15.5))))
    socc = root.occurrences.addNewComponent(F(sp[0]))
    socc.component.name = 'ZZ MG90S'
    socc.component.bRepBodies.add(sv)
    made = [socc]
    for M in sp[1:]:
        o2 = root.occurrences.addExistingComponent(socc.component, adsk.core.Matrix3D.create())
        o2.transform = F(M)
        made.append(o2)
    for o in made:
        b = o.boundingBox
        print('servo', [round(v * 10, 1) for v in (b.minPoint.x, b.minPoint.y, b.minPoint.z, b.maxPoint.x, b.maxPoint.y, b.maxPoint.z)])
    print('occurrences', root.occurrences.count)
    app.activeViewport.fit()
