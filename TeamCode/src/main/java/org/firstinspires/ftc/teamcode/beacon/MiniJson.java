package org.firstinspires.ftc.teamcode.beacon;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Minimal strict JSON parser (RFC 8259 subset: objects, arrays, strings, numbers, true, false,
 * null). Exists because org.json is a throwing stub in JVM unit tests and Gson is not on the
 * compile classpath; this is plain Java, so it behaves the same on the phone and in tests.
 * Objects become LinkedHashMap, arrays ArrayList, numbers Double, strings String.
 */
public final class MiniJson {
    private final String s;
    private int pos;

    private MiniJson(String s) {
        this.s = s;
    }

    public static Object parse(String text) throws BeaconException {
        if (text == null) {
            throw new BeaconException("JSON text is null");
        }
        MiniJson p = new MiniJson(text);
        p.skipWs();
        Object v = p.value(0);
        p.skipWs();
        if (p.pos != p.s.length()) {
            throw p.err("unexpected text after the JSON value");
        }
        return v;
    }

    private BeaconException err(String msg) {
        int line = 1, col = 1;
        for (int i = 0; i < pos && i < s.length(); i++) {
            if (s.charAt(i) == '\n') {
                line++;
                col = 1;
            } else {
                col++;
            }
        }
        return new BeaconException("JSON error at line " + line + ", column " + col + ": " + msg);
    }

    /** Skips whitespace and {@code //} line comments (an extension to JSON, so students can
     *  annotate beacon.json and copy the commented example from the BeaconConfig Javadoc). */
    private void skipWs() {
        while (pos < s.length()) {
            char c = s.charAt(pos);
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                pos++;
            } else if (c == '/' && pos + 1 < s.length() && s.charAt(pos + 1) == '/') {
                while (pos < s.length() && s.charAt(pos) != '\n') {
                    pos++;
                }
            } else {
                break;
            }
        }
    }

    private Object value(int depth) throws BeaconException {
        if (depth > 32) {
            throw err("nesting too deep");
        }
        if (pos >= s.length()) {
            throw err("unexpected end of file");
        }
        char c = s.charAt(pos);
        if (c == '{') {
            return object(depth);
        } else if (c == '[') {
            return array(depth);
        } else if (c == '"') {
            return string();
        } else if (c == 't') {
            literal("true");
            return Boolean.TRUE;
        } else if (c == 'f') {
            literal("false");
            return Boolean.FALSE;
        } else if (c == 'n') {
            literal("null");
            return null;
        } else if (c == '-' || (c >= '0' && c <= '9')) {
            return number();
        }
        throw err("unexpected character '" + c + "'");
    }

    private void literal(String word) throws BeaconException {
        if (!s.startsWith(word, pos)) {
            throw err("expected " + word);
        }
        pos += word.length();
    }

    private Map<String, Object> object(int depth) throws BeaconException {
        Map<String, Object> m = new LinkedHashMap<String, Object>();
        pos++; // {
        skipWs();
        if (pos < s.length() && s.charAt(pos) == '}') {
            pos++;
            return m;
        }
        while (true) {
            skipWs();
            if (pos >= s.length() || s.charAt(pos) != '"') {
                throw err("expected a quoted key");
            }
            String key = string();
            skipWs();
            if (pos >= s.length() || s.charAt(pos) != ':') {
                throw err("expected ':' after key \"" + key + "\"");
            }
            pos++;
            skipWs();
            if (m.containsKey(key)) {
                throw err("duplicate key \"" + key + "\"");
            }
            m.put(key, value(depth + 1));
            skipWs();
            if (pos >= s.length()) {
                throw err("unexpected end of file inside an object");
            }
            char c = s.charAt(pos++);
            if (c == '}') {
                return m;
            }
            if (c != ',') {
                pos--;
                throw err("expected ',' or '}'");
            }
        }
    }

    private List<Object> array(int depth) throws BeaconException {
        List<Object> a = new ArrayList<Object>();
        pos++; // [
        skipWs();
        if (pos < s.length() && s.charAt(pos) == ']') {
            pos++;
            return a;
        }
        while (true) {
            skipWs();
            a.add(value(depth + 1));
            skipWs();
            if (pos >= s.length()) {
                throw err("unexpected end of file inside an array");
            }
            char c = s.charAt(pos++);
            if (c == ']') {
                return a;
            }
            if (c != ',') {
                pos--;
                throw err("expected ',' or ']'");
            }
        }
    }

    private String string() throws BeaconException {
        pos++; // opening quote
        StringBuilder sb = new StringBuilder();
        while (pos < s.length()) {
            char c = s.charAt(pos++);
            if (c == '"') {
                return sb.toString();
            }
            if (c == '\\') {
                if (pos >= s.length()) {
                    break;
                }
                char e = s.charAt(pos++);
                switch (e) {
                    case '"': sb.append('"'); break;
                    case '\\': sb.append('\\'); break;
                    case '/': sb.append('/'); break;
                    case 'b': sb.append('\b'); break;
                    case 'f': sb.append('\f'); break;
                    case 'n': sb.append('\n'); break;
                    case 'r': sb.append('\r'); break;
                    case 't': sb.append('\t'); break;
                    case 'u':
                        if (pos + 4 > s.length()) {
                            throw err("bad \\u escape");
                        }
                        try {
                            sb.append((char) Integer.parseInt(s.substring(pos, pos + 4), 16));
                        } catch (NumberFormatException ex) {
                            throw err("bad \\u escape");
                        }
                        pos += 4;
                        break;
                    default:
                        pos--;
                        throw err("bad escape '\\" + e + "'");
                }
            } else if (c < 0x20) {
                pos--;
                throw err("control character in string");
            } else {
                sb.append(c);
            }
        }
        throw err("unterminated string");
    }

    private Double number() throws BeaconException {
        int start = pos;
        if (s.charAt(pos) == '-') {
            pos++;
        }
        int digitsStart = pos;
        while (pos < s.length() && Character.isDigit(s.charAt(pos))) {
            pos++;
        }
        if (pos == digitsStart) {
            throw err("bad number");
        }
        if (s.charAt(digitsStart) == '0' && pos - digitsStart > 1) {
            throw err("leading zero in number");
        }
        if (pos < s.length() && s.charAt(pos) == '.') {
            pos++;
            int f = pos;
            while (pos < s.length() && Character.isDigit(s.charAt(pos))) {
                pos++;
            }
            if (pos == f) {
                throw err("bad number");
            }
        }
        if (pos < s.length() && (s.charAt(pos) == 'e' || s.charAt(pos) == 'E')) {
            pos++;
            if (pos < s.length() && (s.charAt(pos) == '+' || s.charAt(pos) == '-')) {
                pos++;
            }
            int e = pos;
            while (pos < s.length() && Character.isDigit(s.charAt(pos))) {
                pos++;
            }
            if (pos == e) {
                throw err("bad number");
            }
        }
        try {
            return Double.valueOf(s.substring(start, pos));
        } catch (NumberFormatException ex) {
            pos = start;
            throw err("bad number");
        }
    }
}
