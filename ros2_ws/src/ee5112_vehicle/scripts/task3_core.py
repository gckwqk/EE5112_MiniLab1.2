# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Pure-Python Task 3 command, geometry and mission state logic.

Developed with AI coding assistance.
"""
import bisect
import math
import re

COLOURS = ('Red', 'Orange', 'Yellow', 'Green', 'Blue', 'Purple', 'Black')


def parse_command(text, allow_bare_list=False):
    if not isinstance(text, str) or not text.strip() or not text.isascii():
        raise ValueError('Use an English command, e.g. find red and blue.')
    text = text.strip().lower().rstrip('.!?').strip()
    prefix = re.match(r'^(?:please\s+)?(?:find|locate|search\s+for)\s+', text)
    if prefix:
        text = text[prefix.end():]
    elif not allow_bare_list:
        raise ValueError('Start with find, e.g. find red and blue.')
    colour = '(?:' + '|'.join(c.lower() for c in COLOURS) + ')'
    item = r'(?:the\s+)?' + colour + r'(?:\s+(?:block|cube))?'
    separator = r'(?:\s*,\s*(?:and\s+)?|\s+and\s+|\s+then\s+)'
    if re.fullmatch(item + '(?:' + separator + item + ')*', text) is None:
        raise ValueError('Use only the seven allowed colours in an English list.')
    names = [c.title() for c in re.findall(r'\b' + colour + r'\b', text)]
    if not 1 <= len(names) <= 4:
        raise ValueError('Name between 1 and 4 colours.')
    if len(set(names)) != len(names):
        raise ValueError('Each colour must be distinct.')
    return names


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def rear_pose(centre_x, centre_y, yaw, rear_offset=0.10):
    return (centre_x - rear_offset * math.cos(yaw),
            centre_y - rear_offset * math.sin(yaw), yaw)


def rectangle_collision(x, y, yaw, half_length, half_width, rect):
    """Separating-axis test: a rotated footprint against an axis-aligned box."""
    rx, ry, width, height = rect
    dx, dy = rx + width / 2 - x, ry + height / 2 - y
    c, s = math.cos(yaw), math.sin(yaw)
    axes = ((1., 0.), (0., 1.), (c, s), (-s, c))
    for ax, ay in axes:
        a = half_length * abs(ax*c + ay*s) + half_width * abs(-ax*s + ay*c)
        b = width / 2 * abs(ax) + height / 2 * abs(ay)
        if abs(dx*ax + dy*ay) > a + b:
            return False
    return True


def rectangle_clearance(x, y, yaw, half_length, half_width, rect):
    """Conservative distance between a footprint and a rectangle, zero if touching.

    Separating-axis gaps are lower bounds on the Euclidean separation. Taking
    the largest gap is inexpensive enough to evaluate along every route; it
    deliberately does not overestimate clearance at obstacle corners.
    """
    rx, ry, width, height = rect
    dx, dy = rx + width/2 - x, ry + height/2 - y
    c, s = math.cos(yaw), math.sin(yaw)
    clearance = 0.
    for ax, ay in ((1., 0.), (0., 1.), (c, s), (-s, c)):
        footprint = half_length*abs(ax*c + ay*s) + half_width*abs(-ax*s + ay*c)
        obstacle = width/2*abs(ax) + height/2*abs(ay)
        clearance = max(clearance, abs(dx*ax + dy*ay) - footprint - obstacle)
    return clearance


def segment_hits_rect(start, end, rect):
    x, y, w, h = rect
    low, high = 0., 1.
    for a, b, lo, hi in ((start[0], end[0], x, x+w), (start[1], end[1], y, y+h)):
        delta = b-a
        if abs(delta) < 1e-12:
            if a < lo or a > hi:
                return False
        else:
            t0, t1 = sorted(((lo-a)/delta, (hi-a)/delta))
            low, high = max(low, t0), min(high, t1)
            if low > high:
                return False
    return True


class Arena:
    def __init__(self, data):
        # Read the supplied platform specification directly; wall x/y are SW corners.
        arena_map = data['map']
        self.blocks = {c: tuple(arena_map['block_positions_xy_m'][c]) for c in COLOURS}
        self.walls = [tuple(r[k] for k in ('x', 'y', 'w', 'h'))
                      for r in (arena_map['outer_walls_axis_aligned']
                                + arena_map['inner_walls_axis_aligned'])]
        self.size = tuple(arena_map['overall_m'])
        self.block_size = float(arena_map['block_size_m'])
        self.block_rooms = dict(arena_map['block_room'])
        self.room_x_ranges = {name: tuple(bounds) for name, bounds in arena_map['room_x_ranges_m'].items()}
        self.room_y_range = tuple(arena_map['room_y_range_m'])
        values = [*self.size, self.block_size]
        values.extend(v for xy in self.blocks.values() for v in xy)
        values.extend(v for rect in self.walls for v in rect)
        if not all(math.isfinite(v) for v in values) or min(self.size) <= 0 or self.block_size <= 0:
            raise ValueError('Invalid arena geometry.')

    def room_at(self, pose):
        """Room containing base_link; home/corridor/inter-room gaps return None."""
        x, y = pose[:2]
        if self.room_y_range[0] <= y <= self.room_y_range[1]:
            for room, (left, right) in self.room_x_ranges.items():
                if left <= x <= right:
                    return room
        return None

    def control_poses(self, centres):
        """Validate clear base_link poses, including corridor retreats, for nav_base."""
        if set(centres) != set(self.room_x_ranges):
            raise ValueError('Configure one control pose for every room.')
        result = {}
        for room, pose in centres.items():
            if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
                raise ValueError(room + ' control pose must be finite [x, y, yaw_rad].')
            x, y, yaw = pose
            # The room names identify departure rooms; their retreat targets
            # may extend into the corridor beyond the room boundaries.
            if not self.pose_free(x, y, yaw):
                raise ValueError(room + ' control pose must have a clear footprint inside the arena.')
            result[room] = rear_pose(x, y, wrap_angle(yaw))
        return result

    def retreat_control_candidates(self, departure_room, controls, current, excluded=()):
        """Own control first, then adjacent rooms favoured by reverse heading.

        Inputs/outputs use nav_base poses. Heading only orders the fallback
        attempts; the planner must still validate a complete reverse route.
        Neighbours come from the physical room order, not tuned control poses.
        """
        rooms = sorted(self.room_x_ranges, key=lambda room: self.room_x_ranges[room][0])
        index = rooms.index(departure_room)
        neighbours = rooms[max(0, index-1):index] + rooms[index+1:index+2]
        x, y, yaw = current

        def reverse_preference(room):
            gx, gy, _ = controls[room]
            error = abs(wrap_angle(math.atan2(gy-y, gx-x) - (yaw+math.pi)))
            return error, math.hypot(gx-x, gy-y), room

        ordered = [departure_room] + sorted(neighbours, key=reverse_preference)
        return [{'id': rooms.index(room), 'control_room': room, 'nav': controls[room]}
                for room in ordered if room not in excluded]

    def block_rectangles(self, padding=.01):
        half = self.block_size/2 + padding
        return {c: (x-half, y-half, 2*half, 2*half) for c, (x, y) in self.blocks.items()}

    def pose_free(self, x, y, yaw):
        # Actual swept-wheel footprint: .30 x .24. Includes Nav2 padding
        # plus .01 m extra endpoint clearance.
        hl, hw = .17, .14
        if not (0 <= x <= self.size[0] and 0 <= y <= self.size[1]):
            return False
        return not any(rectangle_collision(x, y, yaw, hl, hw, r)
                       for r in [*self.walls, *self.block_rectangles().values()])

    def observation_poses(self, colour, current=(0.55, 0.35, 0.0), radius=.45,
                          straight_distance=.20, entry_xy_tolerance=.10,
                          final_approach_enabled=True):
        """Box-facing viewing poses, optionally reserving a final straight.

        Nav2 plans to ``entry_nav``, behind the viewing pose. Reserve the entry
        outer XY tolerance (arrival plus settling margin) and .03 m for the
        accepted planner endpoint error, so the stopped pose still has the
        requested straight distance even on the near side.
        Every nominal approach corridor is checked using the padded footprint.
        With final approach disabled, only the viewing footprint/line of sight
        is filtered here; Nav2 plans all the way to ``nav`` with no entry pose.
        """
        if colour not in self.blocks:
            raise ValueError('Unknown target colour.')
        if not math.isfinite(radius) or not .40 <= radius <= .49:
            raise ValueError('Observation radius must be between .40 and .49 m.')
        if not isinstance(final_approach_enabled, bool):
            raise ValueError('final_approach_enabled must be a boolean.')
        if final_approach_enabled:
            if not math.isfinite(straight_distance) or straight_distance < .05:
                raise ValueError('Final straight approach must be at least .05 m.')
            if not math.isfinite(entry_xy_tolerance) or not 0 < entry_xy_tolerance <= .15:
                raise ValueError('Final entry XY tolerance must be in (0, .15] m.')
        if len(current) != 3 or not all(math.isfinite(v) for v in current):
            raise ValueError('Expected a finite current pose.')
        bx, by = self.blocks[colour]
        poses = []
        occluders = [*self.walls, *[r for c, r in self.block_rectangles(0).items()
                                  if c != colour]]
        entry_distance = straight_distance + entry_xy_tolerance + .03 if final_approach_enabled else 0.
        for i in range(16):
            angle = -math.pi/2 + i * math.pi/8
            x, y = bx + radius*math.cos(angle), by + radius*math.sin(angle)
            yaw = wrap_angle(angle + math.pi)
            c, sn = math.cos(yaw), math.sin(yaw)
            ex, ey = x-entry_distance*c, y-entry_distance*sn
            steps = max(1, math.ceil(entry_distance/.01))
            if any(not self.pose_free(ex+entry_distance*j/steps*c,
                                      ey+entry_distance*j/steps*sn, yaw)
                   for j in range(steps+1)):
                continue
            camera = (x+.14*c, y+.14*sn)
            if any(segment_hits_rect(camera, (bx, by), r) for r in occluders):
                continue
            score = math.hypot(ex-current[0], ey-current[1]) + .06*abs(wrap_angle(yaw-current[2]))
            candidate = {'id': i*4, 'side': i, 'centre': (x, y, yaw),
                         'nav': rear_pose(x, y, yaw), 'score': score}
            if final_approach_enabled:
                candidate['entry_nav'] = rear_pose(ex, ey, yaw)
            poses.append(candidate)
        return sorted(poses, key=lambda p: (p['score'], p['id']))


def validate_final_approach_settings(straight_distance, entry_heading_tolerance,
                                     tracking_heading_limit=None):
    """Shared startup/geometry bounds for a forward-only final approach."""
    if not math.isfinite(straight_distance) or straight_distance < .05:
        raise ValueError('final_approach_straight_m must be finite and at least .05 m.')
    if not math.isfinite(entry_heading_tolerance) or not 0 < entry_heading_tolerance < math.pi/2:
        raise ValueError('final_entry_heading_tolerance_rad must be between 0 and pi/2 '
                         '(90 degrees, exclusive) for a forward approach.')
    if tracking_heading_limit is not None and (
            not math.isfinite(tracking_heading_limit) or
            not entry_heading_tolerance < tracking_heading_limit < math.pi/2):
        raise ValueError('final_heading_limit_rad must exceed final_entry_heading_tolerance_rad '
                         'and be less than pi/2 (90 degrees).')


def linear_target_approach(start, block, observation_radius=.45, straight_distance=.20,
                           heading_tolerance=.26, tracking_extension=.20):
    """A fixed forward line from the measured rear axle toward the box centre.

    Heading is the line's reference heading, not a command to rotate at start.
    The configured entry heading error is accepted for the tracker to correct while
    moving forward; exact alignment is not an entry condition. Collision checks
    are required before following this path. There are no arcs, cusps or reverse sections.
    ``tracking_points`` extends the reference beyond the physical stopping point
    so the tracker retains a forward lookahead. The mission must stop at the end
    of ``points``; the extension is reference geometry, not additional travel.
    """
    validate_final_approach_settings(straight_distance, heading_tolerance)
    if (len(start) != 3 or len(block) != 2 or
            not all(math.isfinite(v) for v in (*start, *block, observation_radius,
                                               straight_distance, heading_tolerance, tracking_extension)) or
            not .40 <= observation_radius <= .49 or not .05 <= tracking_extension <= .25):
        raise ValueError('invalid_linear_approach_geometry')
    x, y, yaw = start
    dx, dy = block[0]-x, block[1]-y
    distance = math.hypot(dx, dy)
    heading = math.atan2(dy, dx)
    error = wrap_angle(heading-yaw)
    if abs(error) > heading_tolerance:
        raise ValueError('linear_entry_not_aligned')
    length = distance-(observation_radius+.10)
    if length < straight_distance-1e-9:
        raise ValueError('linear_approach_too_short')
    steps = max(1, math.ceil(length/.01))
    points = [(x+length*j/steps*math.cos(heading),
               y+length*j/steps*math.sin(heading), heading) for j in range(steps+1)]
    tail_steps = math.ceil(tracking_extension/.01)
    end = points[-1]
    tracking_points = points+[
        (end[0]+tracking_extension*j/tail_steps*math.cos(heading),
         end[1]+tracking_extension*j/tail_steps*math.sin(heading), heading)
        for j in range(1, tail_steps+1)]
    return {'points': points, 'tracking_points': tracking_points,
            'straight_m': length, 'entry_heading_error_rad': error}


def shortlist_observation_poses(poses, limit=12):
    """Consider different sides before spending the planning budget on headings."""
    if not isinstance(limit, int) or limit < 1:
        raise ValueError('Candidate limit must be a positive integer.')
    by_side = {}
    for pose in sorted(poses, key=lambda p: (p['score'], p['id'])):
        by_side.setdefault(pose['side'], []).append(pose)
    ordered_sides = sorted(by_side, key=lambda side: (by_side[side][0]['score'], side))
    selected = []
    # Round robin: each side's best heading first, then its second best, etc.
    for rank in range(max((len(group) for group in by_side.values()), default=0)):
        for side in ordered_sides:
            if len(by_side[side]) > rank:
                selected.append(by_side[side][rank])
                if len(selected) == limit:
                    return selected
    return selected


def score_route(poses, arena):
    """Score a planned nav_base path in metre-equivalent units.

    ``poses`` contains (x, y, yaw) at the rear axle. Longer reverse motion and
    gear changes cost more; a short detour with clearance can beat a tight route.
    The known geometry is also sampled every <=.02 m / .08 rad with the Nav2
    padded footprint (.32 x .26 m). This is an additional check; Nav2 must still
    check the live costmap while following the selected route.

    Raises ValueError for an empty, non-finite, malformed or colliding path.
    ``clearance_m`` is a conservative minimum footprint clearance; the near-wall
    penalty is integrated over distance rather than counting path samples.
    """
    try:
        poses = [tuple(pose) for pose in poses]
        valid = bool(poses) and all(len(pose) == 3 and
                                   all(math.isfinite(v) for v in pose) for pose in poses)
    except (TypeError, ValueError):
        valid = False
    if not valid:
        raise ValueError('Expected a nonempty sequence of finite (x, y, yaw) poses.')
    obstacles = [*arena.walls, *arena.block_rectangles().values()]

    def clearance_at(pose):
        nx, ny, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = nx + .10*c, ny + .10*s
        extent_x, extent_y = .16*abs(c)+.13*abs(s), .16*abs(s)+.13*abs(c)
        gap = min(x-extent_x, y-extent_y,
                  arena.size[0]-x-extent_x, arena.size[1]-y-extent_y)
        if gap <= 0:
            raise ValueError('Planned footprint leaves the arena.')
        for rect in obstacles:
            gap = min(gap, rectangle_clearance(x, y, yaw, .16, .13, rect))
            if gap <= 0:
                raise ValueError('Planned footprint intersects a known obstacle.')
        return gap

    clearance = clearance_at(poses[0])
    length = reverse = near_wall = 0.
    cusps, last_gear = 0, None
    for start, end in zip(poses, poses[1:]):
        dx, dy = end[0]-start[0], end[1]-start[1]
        distance = math.hypot(dx, dy)
        turn = wrap_angle(end[2]-start[2])
        length += distance
        if distance > 1e-6:
            # Average heading is robust to sparse curved segments and +/-pi.
            middle_yaw = start[2] + turn/2
            dot = dx*math.cos(middle_yaw) + dy*math.sin(middle_yaw)
            gear = 1 if dot >= 0 else -1
            if gear < 0:
                reverse += distance
            if last_gear is not None and gear != last_gear:
                cusps += 1
            last_gear = gear
        steps = max(1, math.ceil(distance/.02), math.ceil(abs(turn)/.08))
        previous_gap = clearance_at(start)
        for step in range(1, steps+1):
            fraction = step/steps
            sample = (start[0]+fraction*dx, start[1]+fraction*dy,
                      start[2]+fraction*turn)
            gap = clearance_at(sample)
            clearance = min(clearance, gap)
            # Trapezoidal integration makes the score insensitive to path density.
            penalty0 = max(0., (.10-previous_gap)/.10)**2
            penalty1 = max(0., (.10-gap)/.10)**2
            near_wall += distance/steps * (penalty0+penalty1)/2
            previous_gap = gap
    return {'score': length+1.5*reverse+.75*cusps+near_wall,
            'length_m': length, 'reverse_m': reverse, 'cusps': cusps,
            'clearance_m': clearance}


def direction_legs(poses):
    """Inclusive index ranges, preserving every waypoint and shared cusp.

    Pose yaw is the vehicle heading, including in reverse. Duplicate positions
    do not create gears. No smoothing or heading reversal is applied.
    """
    if not poses or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in poses):
        raise ValueError('Expected finite route poses.')
    legs, first, gear, length = [], 0, None, 0.
    for i, (a, b) in enumerate(zip(poses, poses[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            if abs(wrap_angle(b[2]-a[2])) > .01:
                raise ValueError('in_place_heading_change_in_path')
            continue
        yaw = a[2]+wrap_angle(b[2]-a[2])/2
        next_gear = 1 if dx*math.cos(yaw)+dy*math.sin(yaw) >= 0 else -1
        if gear is not None and next_gear != gear:
            legs.append({'first': first, 'last': i, 'gear': gear, 'length_m': length})
            first, length = i, 0.
        gear = next_gear
        length += distance
    legs.append({'first': first, 'last': len(poses)-1, 'gear': gear or 1, 'length_m': length})
    return legs


def dubins_execution_points(poses, gear=1):
    """Execute a forward Dubins plan either forwards or backwards.

    For gear -1 the caller requested control pose -> actual robot pose.
    Reverse the ordering, preserving every body yaw and therefore the same
    asymmetric rear-axle footprint. Never add pi to these orientations.
    """
    legs = direction_legs(poses)
    if gear not in (-1, 1):
        raise ValueError('Dubins execution gear must be +1 or -1.')
    if len(legs) != 1 or legs[0]['gear'] != 1 or legs[0]['length_m'] <= 1e-6:
        raise ValueError('planner_path_not_forward_dubins_check_motion_model')
    return list(poses if gear > 0 else reversed(poses))


class RouteProgress:
    """Monotonic arc progress, matched only to the nearby future of one route.

    A global closest-point search can jump to a much later leg at a crossing or
    interpret repeated forward/back motion as useful progress. Keep the search
    within ``search_ahead`` metres of the last match and refuse matches farther
    than .10 m from the route. Equal-distance matches use the earlier arc point.
    A reversing leg is unavailable until the vehicle has reached its cusp.
    """
    def __init__(self, nav_poses, search_ahead=.35):
        try:
            poses = [tuple(pose) for pose in nav_poses]
            valid = bool(poses) and all(len(pose) == 3 and
                                       all(math.isfinite(v) for v in pose) for pose in poses)
        except (TypeError, ValueError):
            valid = False
        if not valid or not math.isfinite(search_ahead) or search_ahead <= 0:
            raise ValueError('Expected finite route poses and positive search distance.')
        self.search_ahead = search_ahead
        self.progress = 0.
        self.length = 0.
        self._segments, self._ends = [], []
        self._cusps, self._next_cusp = [], 0
        previous_gear = None
        for start, end in zip(poses, poses[1:]):
            dx, dy = end[0]-start[0], end[1]-start[1]
            distance = math.hypot(dx, dy)
            if distance <= 1e-9:
                continue
            middle_yaw = start[2] + wrap_angle(end[2]-start[2])/2
            gear = 1 if dx*math.cos(middle_yaw)+dy*math.sin(middle_yaw) >= 0 else -1
            if previous_gear is not None and gear != previous_gear:
                self._cusps.append((self.length, start[0], start[1]))
            previous_gear = gear
            self._segments.append((start[0], start[1], dx, dy, distance, self.length))
            self.length += distance
            self._ends.append(self.length)

    def update(self, x, y):
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError('Expected finite current route position.')
        window_end = min(self.length, self.progress+self.search_ahead)
        cusp = (self._cusps[self._next_cusp]
                if self._next_cusp < len(self._cusps) else None)
        if cusp is not None:
            # A later reversing leg may occupy the same XY line. It must not
            # win nearest-point matching while the vehicle is still approaching
            # the cusp; otherwise backwards jitter looks like useful progress.
            window_end = min(window_end, cusp[0])
        best_distance2, best_arc = float('inf'), self.progress
        first = bisect.bisect_left(self._ends, self.progress)
        for sx, sy, dx, dy, distance, start_arc in self._segments[first:]:
            if start_arc > window_end:
                break
            low = max(0., (self.progress-start_arc)/distance)
            high = min(1., (window_end-start_arc)/distance)
            fraction = max(low, min(high, ((x-sx)*dx+(y-sy)*dy)/(distance*distance)))
            distance2 = (x-sx-fraction*dx)**2 + (y-sy-fraction*dy)**2
            arc = start_arc+fraction*distance
            if (distance2 < best_distance2-1e-12 or
                    (abs(distance2-best_distance2) <= 1e-12 and arc < best_arc)):
                best_distance2, best_arc = distance2, arc
        if best_distance2 <= .10**2:
            self.progress = max(self.progress, best_arc)
            if (cusp is not None and cusp[0]-self.progress <= .03+1e-9
                    and math.hypot(x-cusp[1], y-cusp[2]) <= .03+1e-9):
                self._next_cusp += 1
        return self.progress


def add_blocks_to_map(data, width, height, resolution, origin, blocks, block_size=.08, padding=.01):
    """Preserve the localization map; add known blocks to a navigation-only copy."""
    if (width <= 0 or height <= 0 or len(data) != width*height or
            not math.isfinite(resolution) or resolution <= 0 or
            not all(math.isfinite(v) for v in origin) or abs(origin[2]) > 1e-6):
        raise ValueError('Expected a valid axis-aligned map in the arena coordinates.')
    output = list(data)
    ox, oy, _ = origin
    half = block_size/2 + padding
    for x, y in blocks.values():
        # Mark every cell that overlaps a padded cube, conservatively.
        x0, x1 = math.floor((x-half-ox)/resolution), math.ceil((x+half-ox)/resolution)
        y0, y1 = math.floor((y-half-oy)/resolution), math.ceil((y+half-oy)/resolution)
        if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
            raise ValueError('A configured block lies outside /map; check map origin and arena.')
        for row in range(y0, y1):
            output[row*width+x0:row*width+x1] = [100]*(x1-x0)
    return output


def controller_minimum_radius(vehicle, max_steering_deg):
    """Rear-axle radius limited by the INNER wheel, independent of planning.

    The supplied URDF limits each real front joint, not a virtual bicycle angle.
    Read wheelbase/track and the physical angle bound from the vehicle spec.
    """
    wheelbase, track, physical_angle = (vehicle[k] for k in
                                      ('wheelbase_m', 'track_m', 'delta_max_deg'))
    if (not all(math.isfinite(x) for x in (wheelbase, track, physical_angle, max_steering_deg)) or
            wheelbase <= 0 or track <= 0 or not 0 < max_steering_deg <= physical_angle < 90):
        raise ValueError('controller_max_steering_deg must be positive and no greater than '
                         'the vehicle steering-joint limit; wheelbase and track must be positive.')
    return track/2 + wheelbase/math.tan(math.radians(max_steering_deg))


def bounded_velocity(v, w, max_speed, minimum_radius):
    # The caller supplies a physical control bound, never the planner radius.
    if not all(math.isfinite(x) and x > 0 for x in (max_speed, minimum_radius)):
        raise ValueError('Velocity limits must be finite and positive.')
    if not (math.isfinite(v) and math.isfinite(w)):
        raise ValueError('Non-finite navigation velocity.')
    if abs(v) < 1e-6:
        return 0., 0.  # No spin commands on this Ackermann platform.
    curvature = max(-1/minimum_radius, min(1/minimum_radius, w/v))
    v = max(-max_speed, min(max_speed, v))
    return v, v*curvature


class Mission:
    """One live Nav2 goal; cancellation/settling finishes before any next goal."""
    TERMINAL = ('IDLE', 'SUCCEEDED', 'FAILED', 'LOCKED')

    def __init__(self):
        self.phase = 'IDLE'
        self.mission_id = 0
        self.sequence = 0
        self.token = None
        self.colours = []
        self.found = []
        self.index = 0
        self.reason = ''
        self.outcome = None
        self.started_ros = self.started_wall = 0.
        self.goal_ros = self.goal_wall = self.observe_ros = self.stop_wall = 0.

    @property
    def active(self):
        return self.phase not in self.TERMINAL

    @property
    def target(self):
        return self.colours[self.index] if self.index < len(self.colours) else None

    @property
    def can_start(self):
        return self.phase in ('IDLE', 'SUCCEEDED', 'FAILED') and self.token is None

    def start(self, colours, ros_now, wall_now):
        if not self.can_start:
            raise ValueError('A mission is active or the node is locked awaiting navigation cleanup.')
        if (not 1 <= len(colours) <= 4 or len(set(colours)) != len(colours) or
                any(c not in COLOURS for c in colours)):
            raise ValueError('Expected 1 to 4 distinct allowed colours.')
        self.mission_id += 1
        self.colours, self.found, self.index = list(colours), [], 0
        self.started_ros, self.started_wall = ros_now, wall_now
        self.reason, self.outcome = '', None
        self.phase = 'NEED_GOAL'

    def planning(self):
        if self.phase != 'NEED_GOAL' or self.token is not None:
            raise RuntimeError('Cannot plan during an active navigation goal.')
        self.phase = 'PLANNING'

    def sending(self, ros_now, wall_now):
        if self.phase not in ('NEED_GOAL', 'PLANNING', 'NEED_LEG') or self.token is not None:
            raise RuntimeError('Cannot overlap navigation goals.')
        continuation = self.phase == 'NEED_LEG'
        # Each goal has its own outcome. A previous aborted attempt must not
        # be reported as the reason for this goal's later successful result.
        self.reason, self.outcome = '', None
        self.sequence += 1
        self.token = (self.mission_id, self.sequence)
        self.phase = 'SENDING'
        if not continuation:
            self.goal_ros = ros_now
        self.goal_wall = wall_now
        return self.token

    def accepted(self, token, accepted, wall_now):
        if token != self.token:
            return False
        if not accepted:
            self.token = None
            if self.phase == 'SENDING':
                self.stop('retry', 'goal_rejected', wall_now)
        elif self.phase == 'SENDING':
            self.phase = 'NAVIGATING'
        return True

    def navigation_result(self, token, status, ros_now, wall_now):
        if token != self.token:
            return False
        self.token = None
        if self.phase in ('STOPPING', 'LOCKED'):
            return True
        if status == 'succeeded':
            self.phase = 'OBSERVING'
            self.observe_ros = ros_now
        else:
            self.stop('retry', 'navigation_' + status, wall_now)
        return True

    def confirm(self, colour, wall_now):
        eligible = (self.phase in ('NEED_GOAL', 'PLANNING', 'NEED_LEG', 'NAVIGATING', 'OBSERVING')
                    or (self.phase == 'STOPPING' and self.outcome in ('next_leg', 'observe')))
        if (not eligible
                or colour != self.target):
            return False
        self.found.append(colour)
        self.stop('advance', '', wall_now)
        return True

    def stop(self, outcome, reason, wall_now):
        if self.phase == 'LOCKED':
            return
        if self.phase != 'STOPPING':
            self.stop_wall = wall_now
        # A fault/cancel cannot be overwritten by a late successful result.
        if self.outcome != 'fail' or outcome == 'fail':
            self.outcome, self.reason = outcome, reason
        self.phase = 'STOPPING'

    def settled(self, ros_now=0.):
        if self.phase != 'STOPPING' or self.token is not None:
            return False
        if self.outcome == 'fail':
            self.phase = 'FAILED'
        elif self.outcome == 'advance':
            self.index += 1
            self.phase = 'SUCCEEDED' if self.index == len(self.colours) else 'NEED_GOAL'
        elif self.outcome == 'next_leg':
            self.phase = 'NEED_LEG'
        elif self.outcome == 'observe':
            self.phase = 'OBSERVING'
            self.observe_ros = ros_now
        else:
            self.phase = 'NEED_GOAL'
        self.outcome = None
        return True

    def lock(self, reason):
        self.phase, self.reason = 'LOCKED', reason
